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

package httprelay

import (
	"io"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"

	"github.com/coder/websocket"
	"github.com/hashicorp/yamux"
)

func TestYamuxConfigValid(t *testing.T) {
	cfg := YamuxConfig()
	if err := yamux.VerifyConfig(cfg); err != nil {
		t.Fatalf("yamux.VerifyConfig(YamuxConfig()) = %v, want nil", err)
	}
	if !cfg.EnableKeepAlive {
		t.Error("YamuxConfig().EnableKeepAlive = false, want true")
	}
	if cfg.KeepAliveInterval != yamuxKeepAliveInterval {
		t.Errorf("YamuxConfig().KeepAliveInterval = %v, want %v", cfg.KeepAliveInterval, yamuxKeepAliveInterval)
	}
	if cfg.MaxStreamWindowSize != yamuxMaxStreamWindowSize {
		t.Errorf("YamuxConfig().MaxStreamWindowSize = %d, want %d", cfg.MaxStreamWindowSize, yamuxMaxStreamWindowSize)
	}
}

func TestYamuxOverCoderWebSocketNetConn(t *testing.T) {
	ctx := t.Context()
	serverErr := make(chan error, 1)

	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		ws, err := websocket.Accept(w, r, &websocket.AcceptOptions{InsecureSkipVerify: true})
		if err != nil {
			serverErr <- err
			return
		}
		sess, err := yamux.Server(websocket.NetConn(ctx, ws, websocket.MessageBinary), YamuxConfig())
		if err != nil {
			serverErr <- err
			return
		}
		defer sess.Close()

		stream, err := sess.Accept()
		if err != nil {
			serverErr <- err
			return
		}
		defer stream.Close()

		if _, err := io.Copy(stream, stream); err != nil {
			serverErr <- err
			return
		}
		serverErr <- nil
	}))
	defer srv.Close()

	wsURL := "ws" + strings.TrimPrefix(srv.URL, "http")
	ws, _, err := websocket.Dial(ctx, wsURL, nil)
	if err != nil {
		t.Fatalf("websocket.Dial(%q) failed: %v", wsURL, err)
	}
	sess, err := yamux.Client(websocket.NetConn(ctx, ws, websocket.MessageBinary), YamuxConfig())
	if err != nil {
		t.Fatalf("yamux.Client() failed: %v", err)
	}
	defer sess.Close()

	stream, err := sess.OpenStream()
	if err != nil {
		t.Fatalf("sess.OpenStream() failed: %v", err)
	}

	payload := strings.Repeat("hello-yamux-websocket-", 4096)
	go func() {
		_, _ = io.WriteString(stream, payload)
	}()

	got := make([]byte, len(payload))
	if _, err := io.ReadFull(stream, got); err != nil {
		t.Fatalf("io.ReadFull(stream) failed: %v", err)
	}
	if string(got) != payload {
		t.Fatalf("echoed %d bytes, want %d", len(got), len(payload))
	}
	_ = stream.Close()

	if err := <-serverErr; err != nil {
		t.Fatalf("server handler failed: %v", err)
	}
}
