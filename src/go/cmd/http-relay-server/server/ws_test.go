// Copyright 2026 The Cloud Robotics Authors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package server

import (
	"context"
	"fmt"
	"io"
	"net"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
	"time"

	"github.com/coder/websocket"
	"github.com/googlecloudrobotics/core/src/go/pkg/httprelay"
	"github.com/hashicorp/yamux"
)

// startWSServer starts a relay server with the WebSocket wire enabled. It
// returns the server, its address and a channel that is closed when the server
// has stopped.
func startWSServer(ctx context.Context, t *testing.T) (*Server, string, <-chan struct{}) {
	t.Helper()
	ln, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatalf("net.Listen() failed: %v", err)
	}
	s := NewServer(Config{WSWire: true})
	stopped := make(chan struct{})
	go func() {
		defer close(stopped)
		s.StartOnListener(ctx, ln)
	}()
	return s, ln.Addr().String(), stopped
}

// connectRelayClient opens a tunnel for the server name "bot" the way
// http-relay-client does, serves handler on it and waits until the relay
// server has registered it.
func connectRelayClient(t *testing.T, s *Server, addr string, handler http.Handler) *yamux.Session {
	t.Helper()
	wsURL := fmt.Sprintf("ws://%s%s?server=bot", addr, wsPath)
	ws, _, err := websocket.Dial(t.Context(), wsURL, nil)
	if err != nil {
		t.Fatalf("websocket.Dial(%q) failed: %v", wsURL, err)
	}
	sess, err := yamux.Server(websocket.NetConn(t.Context(), ws, websocket.MessageBinary), httprelay.YamuxConfig())
	if err != nil {
		t.Fatalf("yamux.Server() failed: %v", err)
	}
	t.Cleanup(func() { sess.Close() })

	// Serve HTTP directly on the yamux session acting as a net.Listener.
	backend := &http.Server{Handler: handler}
	go func() {
		_ = backend.Serve(sess)
	}()
	t.Cleanup(func() { backend.Close() })

	deadline := time.Now().Add(2 * time.Second)
	for !s.HasWSSession("bot") {
		if time.Now().After(deadline) {
			t.Fatal("timed out waiting for the relay server to register the WebSocket session")
		}
		time.Sleep(5 * time.Millisecond)
	}
	return sess
}

func TestWSRelayMissingServerParam(t *testing.T) {
	s := NewServer(Config{WSWire: true})
	req := httptest.NewRequest(http.MethodGet, wsPath, nil)
	rw := httptest.NewRecorder()
	s.Handler().ServeHTTP(rw, req)
	if rw.Code != http.StatusBadRequest {
		t.Errorf("GET %s without server parameter: status = %d, want %d", wsPath, rw.Code, http.StatusBadRequest)
	}
}

func TestWSRelayDisabledReturnsNotFound(t *testing.T) {
	s := NewServer(Config{})
	req := httptest.NewRequest(http.MethodGet, wsPath+"?server=bot", nil)
	rw := httptest.NewRecorder()
	s.Handler().ServeHTTP(rw, req)
	if rw.Code != http.StatusNotFound {
		t.Errorf("GET %s with WSWire=false: status = %d, want %d", wsPath, rw.Code, http.StatusNotFound)
	}
}

func TestWSRelayProxiesHTTPRequestsOverYamux(t *testing.T) {
	s, addr, _ := startWSServer(t.Context(), t)
	if s.HasWSSession("bot") {
		t.Fatal("HasWSSession(\"bot\") = true before connect, want false")
	}
	connectRelayClient(t, s, addr, http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		body, _ := io.ReadAll(r.Body)
		w.Header().Set("X-Backend-Uri", r.RequestURI)
		w.Header().Set("X-Backend-Forwarded-For", r.Header.Get("X-Forwarded-For"))
		w.WriteHeader(http.StatusAccepted)
		fmt.Fprintf(w, "echo:%s", body)
	}))

	targetURL := fmt.Sprintf("http://%s/client/bot/hello/world?a=b", addr)
	req, err := http.NewRequestWithContext(t.Context(), http.MethodPost, targetURL, strings.NewReader("payload"))
	if err != nil {
		t.Fatalf("http.NewRequest(%q) failed: %v", targetURL, err)
	}
	req.Header.Set("X-Forwarded-For", "192.0.2.1")
	resp, err := http.DefaultClient.Do(req)
	if err != nil {
		t.Fatalf("POST %q failed: %v", targetURL, err)
	}
	defer resp.Body.Close()
	gotBody, err := io.ReadAll(resp.Body)
	if err != nil {
		t.Fatalf("io.ReadAll(resp.Body) failed: %v", err)
	}
	if resp.StatusCode != http.StatusAccepted {
		t.Errorf("POST %q status = %d, want %d", targetURL, resp.StatusCode, http.StatusAccepted)
	}
	if got, want := resp.Header.Get("X-Backend-Uri"), "/hello/world?a=b"; got != want {
		t.Errorf("backend request URI = %q, want %q", got, want)
	}
	if got, want := resp.Header.Get("X-Backend-Forwarded-For"), "192.0.2.1"; got != want {
		t.Errorf("backend X-Forwarded-For = %q, want %q", got, want)
	}
	if got, want := string(gotBody), "echo:payload"; got != want {
		t.Errorf("POST %q body = %q, want %q", targetURL, got, want)
	}
}

func TestWSRelayMovesGRPCStatusToTrailers(t *testing.T) {
	s, addr, _ := startWSServer(t.Context(), t)
	// A gRPC Trailers-Only response: the status is in the headers, no body.
	connectRelayClient(t, s, addr, http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "application/grpc")
		w.Header().Set("Grpc-Status", "12")
		w.Header().Set("Grpc-Message", "unimplemented")
	}))

	targetURL := fmt.Sprintf("http://%s/client/bot/pkg.Service/Method", addr)
	resp, err := http.Get(targetURL)
	if err != nil {
		t.Fatalf("GET %q failed: %v", targetURL, err)
	}
	defer resp.Body.Close()
	if _, err := io.Copy(io.Discard, resp.Body); err != nil {
		t.Fatalf("reading body of %q failed: %v", targetURL, err)
	}
	if got := resp.Header.Get("Grpc-Status"); got != "" {
		t.Errorf("header Grpc-Status = %q, want it only in the trailers", got)
	}
	if got, want := resp.Trailer.Get("Grpc-Status"), "12"; got != want {
		t.Errorf("trailer Grpc-Status = %q, want %q", got, want)
	}
	if got, want := resp.Trailer.Get("Grpc-Message"), "unimplemented"; got != want {
		t.Errorf("trailer Grpc-Message = %q, want %q", got, want)
	}
}

func TestWSRelayShutdownAnswersPendingRequest(t *testing.T) {
	ctx, shutdown := context.WithCancel(t.Context())
	defer shutdown()
	s, addr, stopped := startWSServer(ctx, t)
	backendGotRequest := make(chan struct{})
	sess := connectRelayClient(t, s, addr, http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		close(backendGotRequest)
		<-r.Context().Done()
	}))

	status := make(chan int, 1)
	go func() {
		resp, err := http.Get(fmt.Sprintf("http://%s/client/bot/slow", addr))
		if err != nil {
			status <- 0
			return
		}
		resp.Body.Close()
		status <- resp.StatusCode
	}()
	<-backendGotRequest
	shutdown()

	if got := <-status; got != http.StatusBadGateway {
		t.Errorf("status of the pending request after shutdown = %d, want %d", got, http.StatusBadGateway)
	}
	for name, ch := range map[string]<-chan struct{}{"relay server": stopped, "relay client session": sess.CloseChan()} {
		select {
		case <-ch:
		case <-time.After(5 * time.Second):
			t.Errorf("%s still running 5s after shutdown", name)
		}
	}
}

func TestWSRelayClosesSessionWhenStreamCannotBeOpened(t *testing.T) {
	s, addr, _ := startWSServer(t.Context(), t)
	sess := connectRelayClient(t, s, addr, http.HandlerFunc(func(http.ResponseWriter, *http.Request) {}))

	// After GoAway the relay server can no longer open streams, but yamux
	// keeps the session alive on both sides.
	if err := sess.GoAway(); err != nil {
		t.Fatalf("sess.GoAway() failed: %v", err)
	}
	targetURL := fmt.Sprintf("http://%s/client/bot/", addr)
	deadline := time.After(5 * time.Second)
	for {
		// Requests sent before the relay server has seen the GoAway still get
		// a stream, so keep sending until one fails to open.
		resp, err := http.Get(targetURL)
		if err != nil {
			t.Fatalf("GET %q failed: %v", targetURL, err)
		}
		resp.Body.Close()
		select {
		case <-sess.CloseChan():
			return
		case <-deadline:
			t.Fatal("relay client session still open 5s after the relay server could no longer open streams")
		case <-time.After(5 * time.Millisecond):
		}
	}
}
