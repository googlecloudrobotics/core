package main

import (
	"bytes"
	"fmt"
	"io"
	"net/http"
	"strings"
	"testing"
	"testing/synctest"
	"time"

	"github.com/googlecloudrobotics/core/src/go/cmd/http-relay-client/client"
	rt "github.com/googlecloudrobotics/core/src/go/tests/relay"
)

func setupRelay(t *testing.T, serverName string, handler http.Handler) *rt.InMemoryRelayEnv {
	t.Helper()

	config := client.DefaultClientConfig()
	config.ServerName = serverName

	return rt.SetupInMemoryRelay(t, config, handler, rt.SetupRelayOptions{
		SetupHTTPClients: func(relayHTTPClient, backendHTTPClient *http.Client) {
			relayHTTPClient.Transport = rt.NewSwappableTransport(relayHTTPClient.Transport)
		},
	})
}

func expectServerRequest(t *testing.T, serverName string) func(req *http.Request) {
	return func(req *http.Request) {
		t.Helper()
		expectedPath := "/server/request"
		if expectedPath != req.URL.Path {
			t.Errorf("expected request to %q, but got %q", expectedPath, req.URL.Path)
		}
		if server := req.URL.Query().Get("server"); serverName != server {
			t.Errorf("expected server query to contain %q, but got %q", serverName, server)
		}
	}
}

func expectServerResponse(t *testing.T, serverName string) func(req *http.Request) {
	return func(req *http.Request) {
		t.Helper()
		expectedPath := "/server/response"
		if expectedPath != req.URL.Path {
			t.Errorf("expected request to %q, but got %q", expectedPath, req.URL.Path)
		}
		if req.Method != http.MethodPost {
			t.Errorf("expected request method to be %q, but got %q", http.MethodPost, req.Method)
		}
		if server := req.URL.Query().Get("server"); serverName != server {
			t.Errorf("expected server query to be %q, but got %q", serverName, server)
		}
	}
}

// TestMoveRelayServer tests that when a scale up occurs and the reverse proxy moved an existing
// server to a new pod, new user clients can eventually reach the backend.
func TestMoveRelayServer(t *testing.T) {
	synctest.Test(t, func(t *testing.T) {
		expectedResp := []byte("serverA response")
		envA := setupRelay(t, "serverA", http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			w.Write(expectedResp)
		}))

		envB := setupRelay(t, "serverB", http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			w.Write([]byte("serverB response"))
		}))

		// simulate a scale up event and the server is moved to a new pod
		synctest.Wait()
		mockTransport := rt.NewMockTransport(t, envB.Client.Transport)
		envA.Client.Transport.(*rt.SwappableTransport).SwapBackend(mockTransport)

		// user clients should hit the new server, but receive 503 because relay client still on old server.
		mockTransport.ExpectRequest(func(req *http.Request) {
			expectedPath := "/client/serverA"
			if expectedPath != req.URL.Path {
				t.Errorf("expected request to %q, but got %q", expectedPath, req.URL.Path)
			}
		})
		mockTransport.ExpectRequest(expectServerRequest(t, "serverA"))

		resp, err := envA.Client.Get(fmt.Sprint(envA.RelayURL, "/client/serverA"))
		if err != nil {
			t.Fatal(err)
		}
		if resp.StatusCode != http.StatusServiceUnavailable {
			t.Errorf("expected status %d, but got %d", http.StatusServiceUnavailable, resp.StatusCode)
		}
		resp.Body.Close()

		// wait until the relay client times out and reconnect to the new server
		synctest.Sleep(30 * time.Second)

		clientRequestCount := 0
		serverRequestCount := 0
		serverResponseCount := 0
		expectServerOrClientRequest := func(req *http.Request) {
			if strings.HasPrefix(req.URL.Path, "/client") {
				clientRequestCount++
				expectedPath := "/client/serverA"
				if expectedPath != req.URL.Path {
					t.Errorf("expected request to %q, but got %q", expectedPath, req.URL.Path)
				}
			} else if strings.HasPrefix(req.URL.Path, "/server/request") {
				serverRequestCount++
				expectServerRequest(t, "serverA")(req)
			} else if strings.HasPrefix(req.URL.Path, "/server/response") {
				serverResponseCount++
				expectServerResponse(t, "serverA")(req)
			}
		}
		mockTransport.ExpectRequest(expectServerOrClientRequest)
		mockTransport.ExpectRequest(expectServerOrClientRequest)
		mockTransport.ExpectRequest(expectServerOrClientRequest)

		resp, err = envA.Client.Get(fmt.Sprint(envA.RelayURL, "/client/serverA"))
		if err != nil {
			t.Fatal(err)
		}
		if resp.StatusCode != http.StatusOK {
			t.Errorf("expected status %d, but got %d", http.StatusOK, resp.StatusCode)
		}
		body, err := io.ReadAll(resp.Body)
		resp.Body.Close()
		if err != nil {
			t.Fatal(err)
		}
		if !bytes.Equal(expectedResp, body) {
			t.Errorf("expected to receive %q, but got %q", expectedResp, body)
		}

		if clientRequestCount != 1 {
			t.Errorf("expected %d client requests, got %d", 1, clientRequestCount)
		}
		if serverRequestCount != 1 {
			t.Errorf("expected %d server requests, got %d", 1, serverRequestCount)
		}
		if serverResponseCount != 1 {
			t.Errorf("expected %d server responses, got %d", 1, serverResponseCount)
		}
	})
}

// TestMoveRelayServerDuringUserRequest tests that when a scale up occurs and the reverse proxy moved an existing
// server to a new pod, existing user clients can continue to receive data.
func TestMoveRelayServerDuringUserRequest(t *testing.T) {
	t.Skip("not supported")
	synctest.Test(t, func(t *testing.T) {
		sendChunks := make(chan []byte)
		defer close(sendChunks)
		envA := setupRelay(t, "serverA", http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			rc := http.NewResponseController(w)
			w.WriteHeader(200)
			rc.Flush()
			for chunk := range sendChunks {
				if _, err := w.Write(chunk); err != nil {
					return
				}
				if err := rc.Flush(); err != nil {
					return
				}
			}
		}))

		envB := setupRelay(t, "serverB", http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			w.Write([]byte("serverB response"))
		}))

		resp, err := envA.Client.Get(fmt.Sprint(envA.RelayURL, "/client/serverA"))
		if err != nil {
			t.Fatal(err)
		}
		type readResult struct {
			data []byte
			err  error
		}
		body := make(chan readResult)
		go func() {
			buf := make([]byte, 1024)
			defer close(body)
			defer resp.Body.Close()
			for {
				n, err := resp.Body.Read(buf)
				body <- readResult{data: bytes.Clone(buf[:n]), err: err}
				if err != nil {
					break
				}
			}
		}()
		defer func() {
			for range body {
			}
		}()

		data := []byte("chunk 1")
		sendChunks <- data
		select {
		case result := <-body:
			if result.err != nil {
				t.Fatal(result.err)
			}
			if !bytes.Equal(result.data, data) {
				t.Errorf("expected %q to contain %q", result.data, data)
			}
		case <-time.After(time.Second):
			t.Error("did not receive chunk 1")
		}

		// simulate a scale up event and the server is moved to a new pod
		synctest.Wait()
		mockTransport := rt.NewMockTransport(t, envB.Client.Transport)
		envA.Client.Transport.(*rt.SwappableTransport).SwapBackend(mockTransport)

		mockTransport.ExpectRequest(expectServerResponse(t, "serverA"))

		data = []byte("chunk 2")
		sendChunks <- data
		select {
		case result := <-body:
			if result.err != nil {
				t.Fatal(result.err)
			}
			if !bytes.Equal(result.data, data) {
				t.Errorf("expected %q to contain %q", result.data, data)
			}
		case <-time.After(time.Second):
			t.Error("did not receive chunk 2")
		}
	})
}
