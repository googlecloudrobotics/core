package main

import (
	"bytes"
	"context"
	"fmt"
	"io"
	"net"
	"net/http"
	"sync"
	"testing"
	"time"

	"github.com/golang/glog"

	"github.com/googlecloudrobotics/core/src/go/cmd/http-relay-client/client"
	"github.com/googlecloudrobotics/core/src/go/cmd/http-relay-server/server"
)

var (
	relayPort   int // Will be initialized by initRelay()
	backendPort int // Will be initialized by initRelay()
	blockSize   = 10 * 1024
	once        sync.Once
)

func pickUnusedPortOrDie() int {
	var addr *net.TCPAddr
	var err error
	if addr, err = net.ResolveTCPAddr("tcp", "localhost:0"); err == nil {
		var list *net.TCPListener
		if list, err = net.ListenTCP("tcp", addr); err == nil {
			defer list.Close()
			return list.Addr().(*net.TCPAddr).Port
		}
	}
	glog.Fatal("Failed to pick a free TCP port.")
	return 0
}

func initRelay() {
	once.Do(func() {
		glog.Info("Running init.")

		backendPort = pickUnusedPortOrDie()
		relayPort = pickUnusedPortOrDie()

		glog.Infof("Setting up relay.\n\tBackend port: %d\n\tRelay port: %d", backendPort, relayPort)

		go func() {
			relayServer := server.NewServer(server.Config{
				Port:      relayPort,
				BlockSize: blockSize,
			})
			relayServer.Start()
		}()

		go func() {
			config := client.DefaultClientConfig()
			config.RelayScheme = "http"
			config.RelayAddress = fmt.Sprint("127.0.0.1:", relayPort)
			config.BackendScheme = "http"
			config.BackendAddress = fmt.Sprint("127.0.0.1:", backendPort)
			config.DisableAuthForRemote = true
			relayClient := client.NewClient(config)
			relayClient.Start()
		}()

		relayHealthy := false
		deadline := time.Now().Add(5 * time.Second)
		for time.Now().Before(deadline) {
			relayHealthzAddr := fmt.Sprint("http://127.0.0.1:", relayPort, "/healthz")
			res, err := http.Get(relayHealthzAddr)
			if err != nil {
				glog.Infof("Relay server is has not yet started, retrying.")
				time.Sleep(250 * time.Millisecond)
			} else {
				glog.Info("Relay server is up and running.")
				relayHealthy = true
				defer res.Body.Close()
				io.ReadAll(res.Body)
				break
			}
		}
		if !relayHealthy {
			glog.Fatal("Failed to bring up http relay for unknown reason.")
		}
	})
}

func serveFunction(
	f func(w http.ResponseWriter, r *http.Request)) *http.Server {
	return serveFunctionWithTimeout(f, 10*time.Second)
}

func serveFunctionWithTimeout(
	f func(w http.ResponseWriter, r *http.Request),
	handlerTimeout time.Duration) *http.Server {
	httpHandlerFunc := http.HandlerFunc(f)

	srv := &http.Server{
		Addr:         fmt.Sprint("127.0.0.1:", backendPort),
		ReadTimeout:  5 * time.Second, // Time between accepted connection and request body being read.
		WriteTimeout: 5 * time.Second, // Time between request header being read and response body being written.
		Handler:      http.TimeoutHandler(httpHandlerFunc, handlerTimeout, "Timeout"),
	}

	go func() {
		srv.ListenAndServe()
	}()

	return srv
}

func TestHttpResponse(t *testing.T) {
	initRelay()

	expectedResponse := []byte("Unit test response.")

	// Setup a backend function which just serves a string.
	httpServer := serveFunction(func(w http.ResponseWriter, r *http.Request) {
		w.Write(expectedResponse)
	})
	defer httpServer.Shutdown(context.Background())

	// Invoke the backend function through the relay.
	relayAddress := fmt.Sprint("http://127.0.0.1:", relayPort, "/client/server_name/")
	res, err := http.Get(relayAddress)
	if err != nil {
		t.Error("Server responeded with an error. Error %v", err)
	}
	defer res.Body.Close()
	observedResponse, err := io.ReadAll(res.Body)
	if !bytes.Equal(observedResponse, expectedResponse) {
		t.Errorf("Received wrong response.\n\tExpected: %s\n\tObserved: %s", expectedResponse, observedResponse)
	}
}

func TestHttpTimeout(t *testing.T) {
	initRelay()

	// Setup a backend server which will create a timeout and result in a 503.
	httpServer := serveFunctionWithTimeout(func(w http.ResponseWriter, r *http.Request) {
		time.Sleep(2 * time.Second)
	}, 1*time.Second)
	defer httpServer.Shutdown(context.Background())

	// Hit the backend function through the relay and verify that the relay
	// forwards the 503 timeout.
	relayAddress := fmt.Sprint("http://127.0.0.1:", relayPort, "/client/server_name/")
	res, err := http.Get(relayAddress)
	if err != nil {
		t.Errorf("Server responded with an error. Error: %+v", err)
		return
	}
	if res.StatusCode != 503 {
		t.Error("No timeout error received.")
	}
}

func TestHttpErrorPropagation(t *testing.T) {
	initRelay()

	tests := []struct {
		name       string
		statusCode int
	}{
		{"Propagate http.StatusBadRequest", http.StatusBadRequest},                           // 400
		{"Propagate http.StatusUnauthorized", http.StatusUnauthorized},                       // 401
		{"Propagate http.StatusPaymentRequired", http.StatusPaymentRequired},                 // 402
		{"Propagate http.StatusForbidden", http.StatusForbidden},                             // 403
		{"Propagate http.StatusNotFound", http.StatusNotFound},                               // 404
		{"Propagate http.StatusMethodNotAllowed", http.StatusMethodNotAllowed},               // 405
		{"Propagate http.StatusInternalServerError", http.StatusInternalServerError},         // 500
		{"Propagate http.StatusNotImplemented", http.StatusNotImplemented},                   // 501
		{"Propagate http.StatusBadGateway", http.StatusBadGateway},                           // 502
		{"Propagate http.StatusServiceUnavailable", http.StatusServiceUnavailable},           // 503
		{"Propagate http.StatusGatewayTimeout", http.StatusGatewayTimeout},                   // 504
		{"Propagate http.StatusHTTPVersionNotSupported", http.StatusHTTPVersionNotSupported}, // 505
	}

	for _, test := range tests {
		// Invoke a sub-test
		t.Run(test.name, func(t *testing.T) {
			// Setup a backend function which just serves an error code.
			httpServer := serveFunction(func(w http.ResponseWriter, r *http.Request) {
				w.WriteHeader(test.statusCode)
			})
			defer httpServer.Shutdown(context.Background())

			// Invoke the backend function through the relay.
			relayAddress := fmt.Sprint("http://127.0.0.1:", relayPort, "/client/server_name/")
			res, err := http.Get(relayAddress)
			if err != nil {
				t.Errorf("Server responeded with an error. Error %v", err)
			}
			if res.StatusCode != test.statusCode {
				t.Errorf("Server responeded with an unexpected status code.\n\tExpected: %v\n\tObserved: %v", test.statusCode, res.StatusCode)
			}
			defer res.Body.Close()
		})
	}
}
