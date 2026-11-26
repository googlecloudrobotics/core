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

	"github.com/coder/websocket"
	"github.com/googlecloudrobotics/core/src/go/pkg/httprelay"
	"github.com/googlecloudrobotics/ilog"
	"github.com/hashicorp/yamux"
)

const (
	// wsPath is served under /server so existing ingresses route relay-client
	// WebSocket upgrades to http-relay-server without config changes.
	wsPath = "/server/ws"

	// streamCloseTimeout is how long a stream that this side closed may wait
	// for the relay client to close its side before it is reset. This side
	// only closes a stream when it is done with it, so a short timeout frees
	// the relay client's blocked writes and the buffered data of an abandoned
	// request (yamux's default is 5 minutes).
	streamCloseTimeout = 10 * time.Second

	proxyReadBufferSize = 64 * 1024
)

var preservedForwardHeaders = []string{
	"Forwarded",
	"X-Forwarded-For",
	"X-Forwarded-Host",
	"X-Forwarded-Proto",
}

// grpcStatusHeaders carry the status of a gRPC Trailers-Only response.
var grpcStatusHeaders = []string{
	"Grpc-Status",
	"Grpc-Message",
	"Grpc-Status-Details-Bin",
}

type targetKey struct{}

type proxyTarget struct {
	server string
	path   string
	sess   *yamux.Session
}

// wsRelay accepts inbound WebSocket connections from http-relay-client, wraps
// each in a yamux.Client session, and opens yamux streams on demand to proxy
// user-client HTTP requests via httputil.ReverseProxy.
type wsRelay struct {
	proxy *httputil.ReverseProxy

	mu       sync.Mutex
	sessions map[string]*yamux.Session
}

func newWSRelay() *wsRelay {
	w := &wsRelay{
		sessions: make(map[string]*yamux.Session),
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
		ModifyResponse: moveGRPCStatusToTrailers,
		ErrorHandler: func(rw http.ResponseWriter, r *http.Request, err error) {
			if !errors.Is(err, context.Canceled) {
				target, _ := r.Context().Value(targetKey{}).(proxyTarget)
				slog.Error("WebSocket relay proxy error", slog.String("ServerName", target.server), ilog.Err(err))
			}
			// Always answer: the user client can still be connected when the
			// request is cancelled, e.g. because this server is shutting down.
			http.Error(rw, err.Error(), http.StatusBadGateway)
		},
	}
	return w
}

// moveGRPCStatusToTrailers turns a gRPC Trailers-Only response (status in the
// headers, no body) into headers followed by trailers. The proxy flushes the
// headers before it knows that the body is empty, and the user client would
// then get headers plus an empty DATA frame and report "server closed the
// stream without sending trailers" instead of the real status.
func moveGRPCStatusToTrailers(res *http.Response) error {
	if res.Header.Get("Grpc-Status") == "" {
		return nil
	}
	if res.Trailer == nil {
		res.Trailer = http.Header{}
	}
	for _, h := range grpcStatusHeaders {
		if v, ok := res.Header[h]; ok {
			res.Trailer[h] = v
			res.Header.Del(h)
		}
	}
	// A response with a declared length cannot carry trailers over HTTP/1.1.
	res.Header.Del("Content-Length")
	return nil
}

func (w *wsRelay) handleConnect(rw http.ResponseWriter, r *http.Request) {
	server := r.URL.Query().Get("server")
	if server == "" {
		http.Error(rw, "Missing server query parameter", http.StatusBadRequest)
		return
	}

	ws, err := websocket.Accept(rw, r, nil)
	if err != nil {
		slog.Error("Failed to accept relay WebSocket", slog.String("ServerName", server), ilog.Err(err))
		return
	}
	// The request context stays valid until this handler returns and ends when
	// the server shuts down, which closes the tunnel.
	nc := websocket.NetConn(r.Context(), ws, websocket.MessageBinary)
	cfg := httprelay.YamuxConfig()
	cfg.StreamCloseTimeout = streamCloseTimeout
	sess, err := yamux.Client(nc, cfg)
	if err != nil {
		slog.Error("Failed to create yamux session", slog.String("ServerName", server), ilog.Err(err))
		_ = nc.Close()
		return
	}

	w.registerSession(server, sess)
	slog.Info("Relay client connected over WebSocket", slog.String("ServerName", server))

	<-sess.CloseChan()
	w.unregisterSession(server, sess)
	slog.Warn("Relay client WebSocket tunnel closed", slog.String("ServerName", server))
}

func (w *wsRelay) registerSession(server string, sess *yamux.Session) {
	w.mu.Lock()
	defer w.mu.Unlock()
	// A relay client closes its session before it reconnects, so a previous
	// live session belongs to another relay client process with the same name
	// (e.g. during a rolling restart). It is not closed here: it keeps serving
	// its open streams while new requests go to the newest session.
	w.sessions[server] = sess
}

func (w *wsRelay) unregisterSession(server string, sess *yamux.Session) {
	w.mu.Lock()
	defer w.mu.Unlock()
	if w.sessions[server] == sess {
		delete(w.sessions, server)
	}
}

// session returns the live session for server, or nil if it has none.
func (w *wsRelay) session(server string) *yamux.Session {
	w.mu.Lock()
	defer w.mu.Unlock()
	sess := w.sessions[server]
	if sess == nil || sess.IsClosed() {
		return nil
	}
	return sess
}

func (w *wsRelay) dialStream(ctx context.Context, _, _ string) (net.Conn, error) {
	target, ok := ctx.Value(targetKey{}).(proxyTarget)
	if !ok {
		return nil, errors.New("missing proxy target in context")
	}

	stream, err := target.sess.Open()
	if err != nil {
		// yamux can fail to open a stream and keep the session alive, e.g.
		// after a write timeout. Close it so that the relay client reconnects
		// instead of keeping a tunnel that is no longer used.
		go target.sess.Close()
		return nil, fmt.Errorf("opening stream to relay client %q: %w", target.server, err)
	}
	return stream, nil
}

func (w *wsRelay) serveHTTP(ctx context.Context, rw http.ResponseWriter, r *http.Request, backendCtx backendContext, sess *yamux.Session) {
	_ = http.NewResponseController(rw).EnableFullDuplex()

	ctx = context.WithValue(ctx, targetKey{}, proxyTarget{
		server: backendCtx.ServerName,
		path:   backendCtx.Path,
		sess:   sess,
	})
	w.proxy.ServeHTTP(rw, r.WithContext(ctx))
}
