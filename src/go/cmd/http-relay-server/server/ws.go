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
	"errors"
	"fmt"
	"log/slog"
	"net"
	"net/http"
	"net/http/httputil"
	"sync"
	"time"

	"github.com/googlecloudrobotics/core/src/go/pkg/httprelay"
	"github.com/googlecloudrobotics/ilog"
	"github.com/gorilla/websocket"
	"github.com/hashicorp/yamux"
)

const (
	// wsPath is served under /server so existing ingresses route relay-client
	// WebSocket upgrades to http-relay-server without config changes.
	wsPath = "/server/ws"
	// wsQueryParam on /server/request indicates the relay client also holds a
	// WebSocket tunnel, so polling does not clear backendState.ws.
	wsQueryParam = "ws"

	proxyReadBufferSize = 64 * 1024
)

var preservedForwardHeaders = []string{
	"Forwarded",
	"X-Forwarded-For",
	"X-Forwarded-Host",
	"X-Forwarded-Proto",
}

type targetKey struct{}

type proxyTarget struct {
	server string
	path   string
	state  *requestConnState
}

type requestConnState struct {
	mu      sync.Mutex
	aborted bool
	conn    net.Conn
}

func (st *requestConnState) attach(conn net.Conn) bool {
	st.mu.Lock()
	defer st.mu.Unlock()
	if st.aborted {
		_ = conn.Close()
		return false
	}
	st.conn = conn
	return true
}

func (st *requestConnState) close() {
	st.mu.Lock()
	st.aborted = true
	conn := st.conn
	st.mu.Unlock()
	if conn != nil {
		_ = conn.Close()
	}
}

// wsRelay accepts inbound WebSocket connections from http-relay-client, wraps
// each in a yamux.Client session, and opens yamux streams on demand to proxy
// user-client HTTP requests via httputil.ReverseProxy.
type wsRelay struct {
	b        *broker
	upgrader websocket.Upgrader
	proxy    *httputil.ReverseProxy

	mu       sync.Mutex
	sessions map[string]*yamux.Session
	waiters  map[string]chan struct{}
}

func newWSRelay(b *broker) *wsRelay {
	w := &wsRelay{
		b: b,
		upgrader: websocket.Upgrader{
			CheckOrigin: func(*http.Request) bool { return true },
		},
		sessions: make(map[string]*yamux.Session),
		waiters:  make(map[string]chan struct{}),
	}
	transport := &http.Transport{
		DialContext:       w.dialStream,
		DisableKeepAlives: true,
		ReadBufferSize:    proxyReadBufferSize,
	}
	w.proxy = &httputil.ReverseProxy{
		Transport: transport,
		Rewrite: func(pr *httputil.ProxyRequest) {
			target, _ := pr.In.Context().Value(targetKey{}).(proxyTarget)
			for _, h := range preservedForwardHeaders {
				if v, ok := pr.In.Header[h]; ok {
					pr.Out.Header[h] = v
				}
			}
			pr.Out.URL.Scheme = "http"
			pr.Out.URL.Host = target.server
			pr.Out.URL.Path = target.path
			pr.Out.URL.RawPath = ""
			pr.Out.Host = pr.In.Host
		},
		ErrorHandler: func(rw http.ResponseWriter, _ *http.Request, err error) {
			if errors.Is(err, context.Canceled) {
				return
			}
			slog.Error("WebSocket relay proxy error", ilog.Err(err))
			if errors.Is(err, ErrRelayTimeout) {
				http.Error(rw, err.Error(), http.StatusInternalServerError)
				return
			}
			http.Error(rw, err.Error(), http.StatusBadGateway)
		},
	}
	return w
}

func (w *wsRelay) handleConnect(rw http.ResponseWriter, r *http.Request) {
	server := r.URL.Query().Get("server")
	if server == "" {
		http.Error(rw, "Missing server query parameter", http.StatusBadRequest)
		return
	}

	ws, err := w.upgrader.Upgrade(rw, r, nil)
	if err != nil {
		slog.Error("Failed to upgrade relay WebSocket", slog.String("ServerName", server), ilog.Err(err))
		return
	}
	sess, err := yamux.Client(httprelay.NewWSConn(ws), httprelay.YamuxConfig())
	if err != nil {
		slog.Error("Failed to create yamux session", slog.String("ServerName", server), ilog.Err(err))
		_ = ws.Close()
		return
	}

	w.registerSession(server, sess)
	w.b.SetWS(server, true)
	slog.Info("Relay client connected over WebSocket", slog.String("ServerName", server))

	select {
	case <-sess.CloseChan():
	case <-r.Context().Done():
		_ = sess.Close()
	}
	w.unregisterSession(server, sess)
}

func (w *wsRelay) registerSession(server string, sess *yamux.Session) {
	w.mu.Lock()
	defer w.mu.Unlock()
	// Swap in the new session without closing any previous one so in-flight
	// streams on an older session can finish draining during reconnects.
	w.sessions[server] = sess
	if ch := w.waiters[server]; ch != nil {
		close(ch)
		delete(w.waiters, server)
	}
}

func (w *wsRelay) unregisterSession(server string, sess *yamux.Session) {
	w.mu.Lock()
	defer w.mu.Unlock()
	if w.sessions[server] == sess {
		delete(w.sessions, server)
	}
}

func (w *wsRelay) sessionOrWait(server string) (*yamux.Session, <-chan struct{}) {
	w.mu.Lock()
	defer w.mu.Unlock()
	if sess := w.sessions[server]; sess != nil && !sess.IsClosed() {
		return sess, nil
	}
	ch := w.waiters[server]
	if ch == nil {
		ch = make(chan struct{})
		w.waiters[server] = ch
	}
	return nil, ch
}

func (w *wsRelay) dialStream(ctx context.Context, _, _ string) (net.Conn, error) {
	target, ok := ctx.Value(targetKey{}).(proxyTarget)
	if !ok {
		return nil, errors.New("missing proxy target in context")
	}

	timer := time.NewTimer(relayTimeout)
	defer timer.Stop()

	for {
		sess, waitCh := w.sessionOrWait(target.server)
		if sess != nil {
			stream, err := sess.Open()
			if err == nil {
				if !target.state.attach(stream) {
					return nil, ctx.Err()
				}
				w.b.TouchBackend(target.server)
				return stream, nil
			}
			w.unregisterSession(target.server, sess)
			continue
		}

		select {
		case <-waitCh:
		case <-timer.C:
			return nil, fmt.Errorf("cannot reach the client %q; check that it's turned on, set up, and connected to the internet; if the network config recently changed, try again in 1-2 minutes (%w)", target.server, ErrRelayTimeout)
		case <-ctx.Done():
			return nil, ctx.Err()
		}
	}
}

func (w *wsRelay) serveHTTP(ctx context.Context, rw http.ResponseWriter, r *http.Request, backendCtx backendContext) {
	_ = http.NewResponseController(rw).EnableFullDuplex()

	state := &requestConnState{}
	stopAbort := context.AfterFunc(ctx, state.close)
	defer func() {
		stopAbort()
		state.close()
	}()

	reqCtx := context.WithValue(ctx, targetKey{}, proxyTarget{
		server: backendCtx.ServerName,
		path:   backendCtx.Path,
		state:  state,
	})
	w.proxy.ServeHTTP(rw, r.WithContext(reqCtx))
}
