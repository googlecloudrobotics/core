package main

import (
	"bytes"
	"fmt"
	"io"
	"net/http"
	"testing"
	"time"

	"github.com/googlecloudrobotics/core/src/go/cmd/http-relay-client/client"
	rt "github.com/googlecloudrobotics/core/src/go/tests/relay"
)

type relayEnv struct {
	rt.RelayEnv
	backendSrv *http.Server
}

func (e *relayEnv) Close() {
	e.backendSrv.Close()
}

func setupRelay(t *testing.T, handler http.Handler) *relayEnv {
	t.Helper()

	config := client.DefaultClientConfig()
	config.RelayScheme = "http"
	config.BackendScheme = "http"
	config.DisableAuthForRemote = true

	env, backendLn := rt.SetupRelay(t, config)

	backendSrv := &http.Server{
		ReadTimeout:  5 * time.Second,
		WriteTimeout: 5 * time.Second,
		Handler:      handler,
	}
	go backendSrv.Serve(backendLn)

	rt.WaitForClient(t, env.RelayPort, "server_name")

	return &relayEnv{
		RelayEnv:   *env,
		backendSrv: backendSrv,
	}
}

func TestHttpResponse(t *testing.T) {
	expectedResponse := []byte("Unit test response.")

	env := setupRelay(t, http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.Write(expectedResponse)
	}))
	defer env.Close()

	// Invoke the backend function through the relay.
	relayAddress := fmt.Sprint("http://127.0.0.1:", env.RelayPort, "/client/server_name/")
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
	// Setup a backend server which will create a timeout and result in a 503.
	handler := http.TimeoutHandler(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		time.Sleep(2 * time.Second)
	}), 1*time.Second, "Timeout")

	env := setupRelay(t, handler)
	defer env.Close()

	// Hit the backend function through the relay and verify that the relay
	// forwards the 503 timeout.
	relayAddress := fmt.Sprint("http://127.0.0.1:", env.RelayPort, "/client/server_name/")
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
			env := setupRelay(t, http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
				w.WriteHeader(test.statusCode)
			}))
			defer env.Close()

			// Invoke the backend function through the relay.
			relayAddress := fmt.Sprint("http://127.0.0.1:", env.RelayPort, "/client/server_name/")
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
