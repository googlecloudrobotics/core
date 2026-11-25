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
	"fmt"
	"log/slog"
	"net"
	"net/http"
	"net/http/httputil"
	"net/url"
	"os"
	"sync"
	"sync/atomic"
	"time"

	"github.com/googlecloudrobotics/core/src/go/pkg/httprelay"
	"github.com/googlecloudrobotics/core/src/go/pkg/telemetry"
	"github.com/googlecloudrobotics/ilog"
	"github.com/gorilla/websocket"
	"github.com/hashicorp/yamux"
	"go.opentelemetry.io/contrib/instrumentation/net/http/otelhttp"
	"golang.org/x/oauth2"
	"golang.org/x/oauth2/google"
)

const (
	wsPath             = "/server/ws"
	wsQueryParam       = "ws"
	wsHandshakeTimeout = 30 * time.Second
	wsReconnectBackoff = time.Second
	wsShutdownTimeout  = 5 * time.Second
)

var preservedForwardHeaders = []string{
	"Forwarded",
	"X-Forwarded-For",
	"X-Forwarded-Host",
	"X-Forwarded-Proto",
}

// wsRelay maintains a persistent WebSocket + yamux.Server tunnel to
// http-relay-server and serves incoming yamux streams via net/http's
// ReverseProxy, completely independent of the HTTP polling path.
type wsRelay struct {
	config      ClientConfig
	dialer      *websocket.Dialer
	tokenSource oauth2.TokenSource
	handler     http.Handler

	connected atomic.Bool
	ready     chan struct{}
	readyOnce sync.Once
}

func newWSRelay(ctx context.Context, cfg ClientConfig, localTransport http.RoundTripper) (*wsRelay, error) {
	var ts oauth2.TokenSource
	if cfg.RelayScheme == "https" && !cfg.DisableAuthForRemote {
		const scope = "https://www.googleapis.com/auth/cloud-platform.read-only"
		var err error
		ts, err = google.DefaultTokenSource(ctx, scope)
		if err != nil {
			return nil, fmt.Errorf("configuring relay WebSocket credentials: %w", err)
		}
	}

	proxy := &httputil.ReverseProxy{
		Transport: localTransport,
		Rewrite: func(pr *httputil.ProxyRequest) {
			for _, h := range preservedForwardHeaders {
				if v, ok := pr.In.Header[h]; ok {
					pr.Out.Header[h] = v
				}
			}
			pr.Out.URL.Scheme = cfg.BackendScheme
			pr.Out.URL.Host = cfg.BackendAddress
			pr.Out.URL.Path = cfg.BackendPath + pr.In.URL.Path
			if pr.In.URL.RawPath != "" && cfg.BackendPath != "" {
				pr.Out.URL.RawPath = cfg.BackendPath + pr.In.URL.RawPath
			}
			if cfg.PreserveHost {
				pr.Out.Host = pr.In.Host
			} else {
				pr.Out.Host = cfg.BackendAddress
			}
		},
		ErrorHandler: func(w http.ResponseWriter, _ *http.Request, err error) {
			if errors.Is(err, context.Canceled) {
				return
			}
			msg := fmt.Sprintf("Backend request failed with error: %v", err)
			slog.Error("BackendRequest", slog.String("Message", msg))
			http.Error(w, msg, http.StatusInternalServerError)
		},
	}

	baseHandler := http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		_ = http.NewResponseController(w).EnableFullDuplex()
		if cfg.AuthenticationTokenFile != "" {
			token, err := os.ReadFile(cfg.AuthenticationTokenFile)
			if err != nil {
				msg := fmt.Sprintf("Failed to read authentication token from %s: %v", cfg.AuthenticationTokenFile, err)
				slog.Error("BackendRequest", slog.String("Message", msg))
				http.Error(w, msg, http.StatusInternalServerError)
				return
			}
			r.Header.Set("Authorization", fmt.Sprintf("Bearer %s", token))
		}
		proxy.ServeHTTP(w, r)
	})

	tracedHandler := otelhttp.NewHandler(
		baseHandler,
		"",
		otelhttp.WithPropagators(telemetry.HTTPPropagator),
		otelhttp.WithSpanNameFormatter(func(_ string, r *http.Request) string {
			return "Recv." + r.URL.Path
		}),
	)

	return &wsRelay{
		config: cfg,
		dialer: &websocket.Dialer{
			Proxy:            http.ProxyFromEnvironment,
			HandshakeTimeout: wsHandshakeTimeout,
		},
		tokenSource: ts,
		handler:     tracedHandler,
		ready:       make(chan struct{}),
	}, nil
}

func (w *wsRelay) markReady() {
	w.readyOnce.Do(func() {
		close(w.ready)
	})
}

// waitReady waits until the first WebSocket dial attempt either connects or
// fails, avoiding a startup race where an HTTP poll registers first.
func (w *wsRelay) waitReady(ctx context.Context) {
	select {
	case <-w.ready:
	case <-ctx.Done():
	}
}

func (w *wsRelay) isConnected() bool {
	return w.connected.Load()
}

func (w *wsRelay) run(ctx context.Context) {
	for {
		if ctx.Err() != nil {
			return
		}
		if err := w.connectAndServe(ctx); err != nil && ctx.Err() == nil {
			slog.Debug("WebSocket relay tunnel disconnected, retrying", ilog.Err(err))
		}
		w.connected.Store(false)
		w.markReady()

		select {
		case <-ctx.Done():
			return
		case <-time.After(wsReconnectBackoff):
		}
	}
}

func (w *wsRelay) connectAndServe(ctx context.Context) error {
	scheme := "ws"
	if w.config.RelayScheme == "https" {
		scheme = "wss"
	}
	query := url.Values{}
	query.Set("server", w.config.ServerName)
	wsURL := (&url.URL{
		Scheme:   scheme,
		Host:     w.config.RelayAddress,
		Path:     w.config.RelayPrefix + wsPath,
		RawQuery: query.Encode(),
	}).String()

	var header http.Header
	if w.tokenSource != nil {
		tok, err := w.tokenSource.Token()
		if err != nil {
			return fmt.Errorf("fetching relay auth token: %w", err)
		}
		header = http.Header{}
		tok.SetAuthHeader(&http.Request{Header: header})
	}

	ws, resp, err := w.dialer.DialContext(ctx, wsURL, header)
	if resp != nil && resp.Body != nil {
		_ = resp.Body.Close()
	}
	if err != nil {
		return err
	}

	sess, err := yamux.Server(httprelay.NewWSConn(ws), httprelay.YamuxConfig())
	if err != nil {
		_ = ws.Close()
		return err
	}
	defer sess.Close()

	w.connected.Store(true)
	w.markReady()

	srv := &http.Server{
		Handler: w.handler,
		BaseContext: func(net.Listener) context.Context {
			return ctx
		},
	}

	serveDone := make(chan struct{})
	go func() {
		select {
		case <-ctx.Done():
			shutdownCtx, cancel := context.WithTimeout(context.Background(), wsShutdownTimeout)
			_ = srv.Shutdown(shutdownCtx)
			cancel()
			_ = sess.Close()
		case <-serveDone:
		}
	}()

	err = srv.Serve(sess)
	close(serveDone)
	if errors.Is(err, http.ErrServerClosed) {
		return nil
	}
	return err
}
