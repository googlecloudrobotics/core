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
	"log/slog"
	"net"
	"net/http"
	"net/http/httputil"
	"sync"

	"github.com/coder/websocket"
	"github.com/googlecloudrobotics/core/src/go/pkg/httprelay"
	"github.com/googlecloudrobotics/ilog"
	"github.com/hashicorp/yamux"
)

const (
	// wsPath is served under /server so existing ingresses route relay-client
	// WebSocket upgrades to http-relay-server without config changes.
	wsPath = "/server/ws"

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
	sess   *yamux.Session
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
	proxy *httputil.ReverseProxy

	mu          sync.Mutex
	sessions    map[string]*yamux.Session
	allSessions map[*yamux.Session]struct{}
}

func newWSRelay() *wsRelay {
	w := &wsRelay{
		sessions:    make(map[string]*yamux.Session),
		allSessions: make(map[*yamux.Session]struct{}),
	}
	transport := &http.Transport{
		DialContext:        w.dialStream,
		DisableKeepAlives:  true,
		DisableCompression: true,
		ReadBufferSize:     proxyReadBufferSize,
	}
	w.proxy = &httputil.ReverseProxy{
		Transport:     transport,
		FlushInterval: -1,
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

	ws, err := websocket.Accept(rw, r, &websocket.AcceptOptions{
		InsecureSkipVerify: true,
	})
	if err != nil {
		slog.Error("Failed to accept relay WebSocket", slog.String("ServerName", server), ilog.Err(err))
		return
	}
	// Do not bind websocket.NetConn to r.Context(), which is invalid after
	// Accept hijacks the underlying connection. Server shutdown closes sessions
	// via closeAll() after active requests drain.
	nc := websocket.NetConn(context.Background(), ws, websocket.MessageBinary)
	sess, err := yamux.Client(nc, httprelay.YamuxConfig())
	if err != nil {
		slog.Error("Failed to create yamux session", slog.String("ServerName", server), ilog.Err(err))
		_ = nc.Close()
		return
	}

	w.registerSession(server, sess)
	slog.Info("Relay client connected over WebSocket", slog.String("ServerName", server))

	<-sess.CloseChan()
	w.unregisterSession(server, sess)
}

func (w *wsRelay) registerSession(server string, sess *yamux.Session) {
	w.mu.Lock()
	defer w.mu.Unlock()
	// Swap in the new session without closing any previous one so in-flight
	// streams on an older session can finish draining during reconnects.
	w.sessions[server] = sess
	w.allSessions[sess] = struct{}{}
}

func (w *wsRelay) unregisterSession(server string, sess *yamux.Session) {
	w.mu.Lock()
	defer w.mu.Unlock()
	if w.sessions[server] == sess {
		delete(w.sessions, server)
	}
	delete(w.allSessions, sess)
}

func (w *wsRelay) hasSession(server string) bool {
	return w.session(server) != nil
}

func (w *wsRelay) session(server string) *yamux.Session {
	w.mu.Lock()
	defer w.mu.Unlock()
	sess := w.sessions[server]
	if sess == nil || sess.IsClosed() {
		return nil
	}
	return sess
}

func (w *wsRelay) closeAll() {
	w.mu.Lock()
	sessions := make([]*yamux.Session, 0, len(w.allSessions))
	for sess := range w.allSessions {
		sessions = append(sessions, sess)
	}
	clear(w.sessions)
	clear(w.allSessions)
	w.mu.Unlock()

	for _, sess := range sessions {
		_ = sess.Close()
	}
}

func (w *wsRelay) dialStream(ctx context.Context, _, _ string) (net.Conn, error) {
	target, ok := ctx.Value(targetKey{}).(proxyTarget)
	if !ok {
		return nil, errors.New("missing proxy target in context")
	}

	stream, err := target.sess.Open()
	if err != nil {
		w.unregisterSession(target.server, target.sess)
		next := w.session(target.server)
		if next == nil || next == target.sess {
			return nil, err
		}
		stream, err = next.Open()
		if err != nil {
			w.unregisterSession(target.server, next)
			return nil, err
		}
	}
	if !target.state.attach(stream) {
		return nil, ctx.Err()
	}
	return stream, nil
}

func (w *wsRelay) serveHTTP(ctx context.Context, rw http.ResponseWriter, r *http.Request, backendCtx backendContext, sess *yamux.Session) {
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
		sess:   sess,
		state:  state,
	})
	w.proxy.ServeHTTP(rw, r.WithContext(reqCtx))
}
