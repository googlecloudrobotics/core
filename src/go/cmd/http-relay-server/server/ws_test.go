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
	"fmt"
	"io"
	"net"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"

	"github.com/googlecloudrobotics/core/src/go/pkg/httprelay"
	"github.com/gorilla/websocket"
	"github.com/hashicorp/yamux"
)

func TestBrokerPreferWSTransitions(t *testing.T) {
	s := NewServer(Config{})
	if s.b.PreferWS("bot") {
		t.Fatal("PreferWS(\"bot\") = true for unknown backend, want false")
	}

	s.b.SetWS("bot", true)
	if !s.b.PreferWS("bot") {
		t.Fatal("PreferWS(\"bot\") = false after SetWS(true), want true")
	}

	// A dual-wire client poll carrying ws=1 must preserve WebSocket preference.
	dualReq := httptest.NewRequest(http.MethodGet, "/server/request?server=bot&ws=1", nil)
	s.serverRequest(httptest.NewRecorder(), dualReq)
	if !s.b.PreferWS("bot") {
		t.Error("PreferWS(\"bot\") = false after /server/request?ws=1, want true")
	}

	// A legacy HTTP-only poll without ws=1 must clear WebSocket preference.
	legacyReq := httptest.NewRequest(http.MethodGet, "/server/request?server=bot", nil)
	s.serverRequest(httptest.NewRecorder(), legacyReq)
	if s.b.PreferWS("bot") {
		t.Error("PreferWS(\"bot\") = true after /server/request without ws=1, want false")
	}
}

func TestWSRelayMissingServerParam(t *testing.T) {
	s := NewServer(Config{})
	req := httptest.NewRequest(http.MethodGet, wsPath, nil)
	rw := httptest.NewRecorder()
	s.ws.handleConnect(rw, req)
	if rw.Code != http.StatusBadRequest {
		t.Errorf("handleConnect(missing server) status = %d, want %d", rw.Code, http.StatusBadRequest)
	}
}

func TestWSRelayProxiesHTTPRequestsOverYamux(t *testing.T) {
	ln, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatalf("net.Listen() failed: %v", err)
	}
	s := NewServer(Config{})
	go s.StartOnListener(t.Context(), ln)

	wsURL := fmt.Sprintf("ws://%s%s?server=bot", ln.Addr().String(), wsPath)
	ws, _, err := websocket.DefaultDialer.DialContext(t.Context(), wsURL, nil)
	if err != nil {
		t.Fatalf("websocket.DialContext(%q) failed: %v", wsURL, err)
	}
	sess, err := yamux.Server(httprelay.NewWSConn(ws), httprelay.YamuxConfig())
	if err != nil {
		ws.Close()
		t.Fatalf("yamux.Server() failed: %v", err)
	}
	defer sess.Close()

	// Serve HTTP directly on the yamux session acting as a net.Listener.
	backendSrv := &http.Server{
		Handler: http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			body, _ := io.ReadAll(r.Body)
			w.Header().Set("X-Backend-Path", r.URL.Path)
			w.WriteHeader(http.StatusAccepted)
			fmt.Fprintf(w, "echo:%s:%s", r.URL.Path, string(body))
		}),
	}
	go func() {
		_ = backendSrv.Serve(sess)
	}()
	defer backendSrv.Close()

	targetURL := fmt.Sprintf("http://%s/client/bot/hello/world", ln.Addr().String())
	resp, err := http.Post(targetURL, "text/plain", strings.NewReader("payload"))
	if err != nil {
		t.Fatalf("http.Post(%q) failed: %v", targetURL, err)
	}
	defer resp.Body.Close()
	gotBody, err := io.ReadAll(resp.Body)
	if err != nil {
		t.Fatalf("io.ReadAll(resp.Body) failed: %v", err)
	}
	if resp.StatusCode != http.StatusAccepted {
		t.Errorf("POST %q status = %d, want %d", targetURL, resp.StatusCode, http.StatusAccepted)
	}
	if got, want := resp.Header.Get("X-Backend-Path"), "/hello/world"; got != want {
		t.Errorf("X-Backend-Path = %q, want %q", got, want)
	}
	if got, want := string(gotBody), "echo:/hello/world:payload"; got != want {
		t.Errorf("POST %q body = %q, want %q", targetURL, got, want)
	}
}
