package main

import (
	"bytes"
	"context"
	"fmt"
	"io"
	"net/http"
	"net/http/httptest"
	"sync"
	"sync/atomic"
	"testing"
	"testing/synctest"
	"time"

	"github.com/googlecloudrobotics/core/src/go/cmd/http-relay-client/client"
	"github.com/googlecloudrobotics/core/src/go/cmd/http-relay-server/server"
	rt "github.com/googlecloudrobotics/core/src/go/tests/relay"
)

func setupRelay(t *testing.T, handler http.Handler) *rt.InMemoryRelayEnv {
	t.Helper()

	config := client.DefaultClientConfig()
	config.RelayScheme = "http"
	config.BackendScheme = "http"
	config.DisableAuthForRemote = true

	return rt.SetupInMemoryRelay(t, config, handler)
}

func TestHttpResponse(t *testing.T) {
	synctest.Test(t, func(t *testing.T) {
		expectedResponse := []byte("Unit test response.")

		env := setupRelay(t, http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			w.Write(expectedResponse)
		}))

		// Invoke the backend function through the relay.
		relayAddress := fmt.Sprint(env.RelayURL, "/client/server_name/")
		res, err := env.Client.Get(relayAddress)
		if err != nil {
			t.Fatalf("Server responded with an error. Error %v", err)
		}
		defer res.Body.Close()
		observedResponse, err := io.ReadAll(res.Body)
		if !bytes.Equal(observedResponse, expectedResponse) {
			t.Errorf("Received wrong response.\n\tExpected: %s\n\tObserved: %s", expectedResponse, observedResponse)
		}
	})
}

func TestHttpTimeout(t *testing.T) {
	synctest.Test(t, func(t *testing.T) {
		// Setup a backend server which will create a timeout and result in a 503.
		handler := http.TimeoutHandler(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			time.Sleep(2 * time.Second)
		}), 1*time.Second, "Timeout")

		env := setupRelay(t, handler)

		// Hit the backend function through the relay and verify that the relay
		// forwards the 503 timeout.
		relayAddress := fmt.Sprint(env.RelayURL, "/client/server_name/")
		res, err := env.Client.Get(relayAddress)
		if err != nil {
			t.Errorf("Server responded with an error. Error: %+v", err)
			return
		}
		res.Body.Close()
		if res.StatusCode != 503 {
			t.Errorf("No timeout error received. Observed: %d", res.StatusCode)
		}
		// Advance fake time so the inner handler goroutine spawned by http.TimeoutHandler finishes its 2s sleep.
		time.Sleep(2 * time.Second)
	})
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
			synctest.Test(t, func(t *testing.T) {
				env := setupRelay(t, http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
					w.WriteHeader(test.statusCode)
				}))

				// Invoke the backend function through the relay.
				relayAddress := fmt.Sprint(env.RelayURL, "/client/server_name/")
				res, err := env.Client.Get(relayAddress)
				if err != nil {
					t.Fatalf("Server responded with an error. Error %v", err)
				}
				defer res.Body.Close()
				if res.StatusCode != test.statusCode {
					t.Errorf("Server responded with an unexpected status code.\n\tExpected: %v\n\tObserved: %v", test.statusCode, res.StatusCode)
				}
			})
		})
	}
}

func TestHttpChunkedRequestBody(t *testing.T) {
	synctest.Test(t, func(t *testing.T) {
		expectedRequestBody := []byte("chunk1-chunk2-chunk3")
		expectedResponseBody := []byte("received: chunk1-chunk2-chunk3")

		env := setupRelay(t, http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			if r.Method != http.MethodPost {
				http.Error(w, "expected POST method", http.StatusMethodNotAllowed)
				return
			}
			body, err := io.ReadAll(r.Body)
			if err != nil {
				http.Error(w, fmt.Sprintf("failed to read request body: %v", err), http.StatusInternalServerError)
				return
			}
			if !bytes.Equal(body, expectedRequestBody) {
				http.Error(w, fmt.Sprintf("unexpected body: got %q, want %q", body, expectedRequestBody), http.StatusBadRequest)
				return
			}
			w.WriteHeader(http.StatusOK)
			w.Write(append([]byte("received: "), body...))
		}))

		// Use an io.Pipe to stream chunks in the request body (Transfer-Encoding: chunked).
		pr, pw := io.Pipe()
		go func() {
			defer pw.Close()
			chunks := [][]byte{[]byte("chunk1-"), []byte("chunk2-"), []byte("chunk3")}
			for _, chunk := range chunks {
				if _, err := pw.Write(chunk); err != nil {
					t.Errorf("failed to write chunk to pipe: %v", err)
					return
				}
			}
		}()

		relayAddress := fmt.Sprint(env.RelayURL, "/client/server_name/upload")
		req, err := http.NewRequestWithContext(t.Context(), http.MethodPost, relayAddress, pr)
		if err != nil {
			t.Fatalf("failed to create request: %v", err)
		}
		req.ContentLength = -1 // Force Transfer-Encoding: chunked

		client := *env.Client
		client.Timeout = 5 * time.Second
		res, err := client.Do(req)
		if err != nil {
			t.Fatalf("request through relay failed: %v", err)
		}
		defer res.Body.Close()

		if res.StatusCode != http.StatusOK {
			respBody, _ := io.ReadAll(res.Body)
			t.Fatalf("unexpected status code %d: %s", res.StatusCode, respBody)
		}

		observedResponse, err := io.ReadAll(res.Body)
		if err != nil {
			t.Fatalf("failed to read response body: %v", err)
		}
		if !bytes.Equal(observedResponse, expectedResponseBody) {
			t.Errorf("received wrong response.\n\tExpected: %s\n\tObserved: %s", expectedResponseBody, observedResponse)
		}
	})
}

type delayedRoundTripper struct {
	base      http.RoundTripper
	halfRTT   time.Duration
	pollCount atomic.Int64
}

func (d *delayedRoundTripper) sleep(ctx context.Context) error {
	timer := time.NewTimer(d.halfRTT)
	defer timer.Stop()
	select {
	case <-ctx.Done():
		return ctx.Err()
	case <-timer.C:
		return nil
	}
}

func (d *delayedRoundTripper) RoundTrip(req *http.Request) (*http.Response, error) {
	if req.URL.Path == "/server/request" {
		d.pollCount.Add(1)
	}
	if err := d.sleep(req.Context()); err != nil {
		return nil, err
	}
	resp, err := d.base.RoundTrip(req)
	if err != nil {
		return nil, err
	}
	if err := d.sleep(req.Context()); err != nil {
		resp.Body.Close()
		return nil, err
	}
	return resp, nil
}

// TestHttpBurstWithHighPollLatency verifies that with a single polling worker
// (NumPendingRequests=1) and a 500ms RTT between relay client and server, a
// burst of 25 concurrent requests completes within ~2 RTTs via batching instead
// of timing out against relayTimeout (10s).
func TestHttpBurstWithHighPollLatency(t *testing.T) {
	synctest.Test(t, func(t *testing.T) {
		relayServer := server.NewServer(server.Config{
			BlockSize: 10 * 1024,
		})
		relayTS := httptest.NewTestServer(t, relayServer.Handler())
		backendTS := httptest.NewTestServer(t, http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			fmt.Fprintf(w, "echo:%s", r.URL.Path)
		}))

		userHTTPClient := relayTS.Client()
		delayedRT := &delayedRoundTripper{
			base:    relayTS.Client().Transport,
			halfRTT: 250 * time.Millisecond, // 500ms RTT
		}
		relayHTTPClient := &http.Client{Transport: delayedRT}
		backendHTTPClient := backendTS.Client()

		config := client.DefaultClientConfig()
		config.RelayScheme = "http"
		config.BackendScheme = "http"
		config.DisableAuthForRemote = true
		config.NumPendingRequests = 1
		config.EnableRequestBatching = true

		relayClient := client.NewClient(config)
		ctx, cancel := context.WithCancel(t.Context())
		var clientWG sync.WaitGroup
		clientWG.Go(func() {
			relayClient.StartWithClients(ctx, relayHTTPClient, backendHTTPClient)
		})
		t.Cleanup(func() {
			synctest.Wait()
			cancel()
			relayTS.CloseClientConnections()
			backendTS.CloseClientConnections()
			clientWG.Wait()
		})

		// Let the single worker finish its initial 250ms outbound leg and park on reqChan.
		time.Sleep(300 * time.Millisecond)
		synctest.Wait()

		const burstSize = 25
		start := time.Now()
		var reqWG sync.WaitGroup
		for i := range burstSize {
			path := fmt.Sprintf("/item/%d", i)
			reqWG.Go(func() {
				res, err := userHTTPClient.Get(relayTS.URL + "/client/server_name" + path)
				if err != nil {
					t.Errorf("request %s failed: %v", path, err)
					return
				}
				defer res.Body.Close()
				body, err := io.ReadAll(res.Body)
				if err != nil {
					t.Errorf("request %s read body: %v", path, err)
					return
				}
				if res.StatusCode != http.StatusOK {
					t.Errorf("request %s status = %d, want 200 (body: %s)", path, res.StatusCode, body)
					return
				}
				if want := "echo:" + path; string(body) != want {
					t.Errorf("request %s body = %q, want %q", path, body, want)
				}
			})
		}
		reqWG.Wait()
		elapsed := time.Since(start)

		// Without batching, 25 requests at 500ms RTT would need 24*500ms = 12s and hit relayTimeout (10s).
		// With batching, req #1 takes ~500ms and reqs #2..#25 drain in a single batch at ~1.0s.
		if elapsed > 2*time.Second {
			t.Errorf("burst of %d requests took %v, want <= 2s", burstSize, elapsed)
		}
		// Expect at most 3 polls (initial parked poll + 1 batch poll + next idle poll).
		if polls := delayedRT.pollCount.Load(); polls > 3 {
			t.Errorf("got %d /server/request polls for burst of %d, want <= 3", polls, burstSize)
		}
	})
}
