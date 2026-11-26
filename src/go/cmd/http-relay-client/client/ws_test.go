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

package client

import (
	"context"
	"errors"
	"net/http"
	"net/http/httptest"
	"strings"
	"sync/atomic"
	"testing"
	"time"
)

func TestWSHandshakeErrorIncludesStatusAndBody(t *testing.T) {
	oldServer := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		http.Error(w, `missing required header "X-Server-Name" for path "/server/ws"`, http.StatusBadRequest)
	}))
	defer oldServer.Close()

	cfg := DefaultClientConfig()
	cfg.RelayScheme = "http"
	cfg.RelayAddress = strings.TrimPrefix(oldServer.URL, "http://")
	cfg.DisableAuthForRemote = true

	wsr, err := newWSRelay(t.Context(), cfg, http.DefaultTransport)
	if err != nil {
		t.Fatalf("newWSRelay() failed: %v", err)
	}

	connectedAt, err := wsr.connectAndServe(t.Context())
	if !connectedAt.IsZero() {
		t.Errorf("connectedAt = %v, want zero time on handshake failure", connectedAt)
	}
	if err == nil {
		t.Fatal("connectAndServe() = nil, want handshake error")
	}

	var hsErr *wsHandshakeError
	if !errors.As(err, &hsErr) {
		t.Fatalf("error type = %T (%v), want *wsHandshakeError", err, err)
	}
	if hsErr.statusCode != http.StatusBadRequest {
		t.Errorf("hsErr.statusCode = %d, want %d", hsErr.statusCode, http.StatusBadRequest)
	}
	if !hsErr.isUnsupported() {
		t.Error("hsErr.isUnsupported() = false for 400 Bad Request, want true")
	}
	if !strings.Contains(err.Error(), "400") || !strings.Contains(err.Error(), "X-Server-Name") {
		t.Errorf("error = %q, want HTTP 400 status and server message", err.Error())
	}
}

func TestWSRelayExponentialBackoffOnUnsupportedServer(t *testing.T) {
	var attempts atomic.Int32
	oldServer := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		attempts.Add(1)
		http.Error(w, "WebSocket relay disabled", http.StatusNotFound)
	}))
	defer oldServer.Close()

	cfg := DefaultClientConfig()
	cfg.RelayScheme = "http"
	cfg.RelayAddress = strings.TrimPrefix(oldServer.URL, "http://")
	cfg.DisableAuthForRemote = true

	wsr, err := newWSRelay(t.Context(), cfg, http.DefaultTransport)
	if err != nil {
		t.Fatalf("newWSRelay() failed: %v", err)
	}
	wsr.initialReconnectDelay = 20 * time.Millisecond
	wsr.maxReconnectDelay = 200 * time.Millisecond

	ctx, cancel := context.WithTimeout(t.Context(), 220*time.Millisecond)
	defer cancel()
	wsr.run(ctx)

	got := attempts.Load()
	// Without exponential backoff, a fixed 20ms retry loop would issue ~11 requests
	// in 220ms; with exponential backoff (20ms -> 40ms -> 80ms -> 160ms, +/-50%
	// jitter) we should only see a handful of attempts.
	if got < 2 || got > 7 {
		t.Errorf("attempts in 220ms = %d, want between 2 and 7 (exponential backoff)", got)
	}
}
