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
	"bytes"
	"fmt"
	"io"
	"net/http"
	"net/http/httptest"
	"strings"
	"sync"
	"testing"

	"github.com/gorilla/websocket"
	"github.com/hashicorp/yamux"
)

func TestWSConnWithYamuxMultiplexing(t *testing.T) {
	upgrader := websocket.Upgrader{}
	serverSessionCh := make(chan *yamux.Session, 1)
	ts := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		ws, err := upgrader.Upgrade(w, r, nil)
		if err != nil {
			t.Errorf("upgrader.Upgrade() failed: %v", err)
			return
		}
		// Match relay architecture: server acts as yamux.Client so it can open
		// streams on demand toward the dialer.
		sess, err := yamux.Client(NewWSConn(ws), YamuxConfig())
		if err != nil {
			t.Errorf("yamux.Client() failed: %v", err)
			ws.Close()
			return
		}
		serverSessionCh <- sess
	}))
	defer ts.Close()

	wsURL := "ws" + strings.TrimPrefix(ts.URL, "http")
	ws, _, err := websocket.DefaultDialer.DialContext(t.Context(), wsURL, nil)
	if err != nil {
		t.Fatalf("websocket.DialContext(%q) failed: %v", wsURL, err)
	}
	clientSession, err := yamux.Server(NewWSConn(ws), YamuxConfig())
	if err != nil {
		ws.Close()
		t.Fatalf("yamux.Server() failed: %v", err)
	}
	defer clientSession.Close()

	// Echo every stream accepted by the WebSocket dialer (relay-client role).
	go func() {
		for {
			stream, err := clientSession.Accept()
			if err != nil {
				return
			}
			go func() {
				defer stream.Close()
				_, _ = io.Copy(stream, stream)
			}()
		}
	}()

	serverSession := <-serverSessionCh
	defer serverSession.Close()

	const numStreams = 4
	payload := bytes.Repeat([]byte("yamux-over-ws-"), 10000) // ~140 KiB per stream
	var wg sync.WaitGroup
	for i := range numStreams {
		wg.Go(func() {
			stream, err := serverSession.Open()
			if err != nil {
				t.Errorf("serverSession.Open() stream %d failed: %v", i, err)
				return
			}
			defer stream.Close()

			want := append([]byte(fmt.Sprintf("stream-%d:", i)), payload...)
			writeErrCh := make(chan error, 1)
			go func() {
				_, err := stream.Write(want)
				writeErrCh <- err
			}()

			got := make([]byte, len(want))
			if _, err := io.ReadFull(stream, got); err != nil {
				t.Errorf("io.ReadFull(stream %d) failed: %v", i, err)
				return
			}
			if err := <-writeErrCh; err != nil {
				t.Errorf("stream.Write(stream %d) failed: %v", i, err)
				return
			}
			if !bytes.Equal(got, want) {
				t.Errorf("stream %d echoed %d bytes mismatching want (%d bytes)", i, len(got), len(want))
			}
		})
	}
	wg.Wait()
}
