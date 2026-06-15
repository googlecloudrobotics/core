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
	relayPort      int
	backendPort    int
	blockSize      = 10 * 1024
	once           sync.Once
	backendHandler http.Handler
	backendMu      sync.RWMutex
)

func setBackendHandler(h http.Handler) {
	backendMu.Lock()
	backendHandler = h
	backendMu.Unlock()
}

func getBackendHandler() http.Handler {
	backendMu.RLock()
	defer backendMu.RUnlock()
	return backendHandler
}

type backendResetter struct{}

func (r backendResetter) Shutdown(ctx context.Context) error {
	setBackendHandler(nil)
	return nil
}

func initRelay() {
	once.Do(func() {
		glog.Info("Running init.")

		// 1. Setup Backend Server
		backendLn, err := net.Listen("tcp", "127.0.0.1:0")
		if err != nil {
			glog.Fatalf("Failed to listen on backend port: %v", err)
		}
		backendPort = backendLn.Addr().(*net.TCPAddr).Port

		backendSrv := &http.Server{
			ReadTimeout:  5 * time.Second,
			WriteTimeout: 5 * time.Second,
			Handler: http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
				if h := getBackendHandler(); h != nil {
					h.ServeHTTP(w, r)
				} else {
					http.Error(w, "No backend handler configured", http.StatusInternalServerError)
				}
			}),
		}
		go backendSrv.Serve(backendLn)

		// 2. Setup Relay Server
		relayLn, err := net.Listen("tcp", "127.0.0.1:0")
		if err != nil {
			glog.Fatalf("Failed to listen on relay port: %v", err)
		}
		relayPort = relayLn.Addr().(*net.TCPAddr).Port

		relayServer := server.NewServer(server.Config{
			BlockSize: blockSize,
		})
		go relayServer.StartOnListener(relayLn)

		// 3. Setup Relay Client
		config := client.DefaultClientConfig()
		config.RelayScheme = "http"
		config.RelayAddress = fmt.Sprint("127.0.0.1:", relayPort)
		config.BackendScheme = "http"
		config.BackendAddress = fmt.Sprint("127.0.0.1:", backendPort)
		config.DisableAuthForRemote = true
		relayClient := client.NewClient(config)
		go relayClient.Start()

		glog.Infof("Setting up relay.\n\tBackend port: %d\n\tRelay port: %d", backendPort, relayPort)

		// 4. Wait for Relay Server to be healthy
		relayHealthy := false
		deadline := time.Now().Add(5 * time.Second)
		for time.Now().Before(deadline) {
			relayHealthzAddr := fmt.Sprint("http://127.0.0.1:", relayPort, "/healthz")
			res, err := http.Get(relayHealthzAddr)
			if err != nil {
				glog.Infof("Relay server has not yet started, retrying.")
				time.Sleep(100 * time.Millisecond)
			} else {
				glog.Info("Relay server is up and running.")
				relayHealthy = true
				res.Body.Close()
				break
			}
		}
		if !relayHealthy {
			glog.Fatal("Failed to bring up http relay server.")
		}

		// 5. Wait for Relay Client to register
		relayAddress := fmt.Sprintf("http://127.0.0.1:%d/client/server_name/", relayPort)
		clientRegistered := false
		deadline = time.Now().Add(5 * time.Second)
		for time.Now().Before(deadline) {
			res, err := http.Get(relayAddress)
			if err == nil {
				body, _ := io.ReadAll(res.Body)
				res.Body.Close()
				if res.StatusCode == http.StatusServiceUnavailable && bytes.Contains(body, []byte("unknown client")) {
					glog.Infof("Relay client has not yet registered, retrying.")
					time.Sleep(100 * time.Millisecond)
					continue
				}
				glog.Info("Relay client registered.")
				clientRegistered = true
				break
			}
			time.Sleep(100 * time.Millisecond)
		}
		if !clientRegistered {
			glog.Fatal("Relay client failed to register.")
		}
	})
}

func serveFunction(
	f func(w http.ResponseWriter, r *http.Request)) interface{ Shutdown(context.Context) error } {
	return serveFunctionWithTimeout(f, 10*time.Second)
}

func serveFunctionWithTimeout(
	f func(w http.ResponseWriter, r *http.Request),
	handlerTimeout time.Duration) interface{ Shutdown(context.Context) error } {

	setBackendHandler(http.TimeoutHandler(http.HandlerFunc(f), handlerTimeout, "Timeout"))
	return backendResetter{}
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
		t.Fatalf("Server responded with an error. Error %v", err)
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
		t.Errorf("No timeout error received. Observed: %d", res.StatusCode)
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
				t.Fatalf("Server responded with an error. Error %v", err)
			}
			defer res.Body.Close()
			if res.StatusCode != test.statusCode {
				t.Errorf("Server responded with an unexpected status code.\n\tExpected: %v\n\tObserved: %v", test.statusCode, res.StatusCode)
			}
		})
	}
}
