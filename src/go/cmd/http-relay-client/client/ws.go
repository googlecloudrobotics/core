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
	"crypto/tls"
	"errors"
	"fmt"
	"io"
	"log/slog"
	"net"
	"net/http"
	"net/http/httputil"
	"net/url"
	"os"
	"strings"
	"time"

	"github.com/cenkalti/backoff/v4"
	"github.com/coder/websocket"
	"github.com/googlecloudrobotics/core/src/go/pkg/httprelay"
	"github.com/googlecloudrobotics/core/src/go/pkg/telemetry"
	"github.com/googlecloudrobotics/ilog"
	"github.com/hashicorp/yamux"
	"go.opentelemetry.io/contrib/instrumentation/net/http/otelhttp"
	"golang.org/x/oauth2"
	"golang.org/x/oauth2/google"
)

const (
	wsPath                  = "/server/ws"
	wsHandshakeTimeout      = 30 * time.Second
	wsInitialReconnectDelay = time.Second
	wsMaxReconnectDelay     = time.Minute
	wsStableSessionDuration = 30 * time.Second
	wsShutdownTimeout       = 5 * time.Second
	wsMaxErrorBodyBytes     = 256
)

var preservedForwardHeaders = []string{
	"Forwarded",
	"X-Forwarded-For",
	"X-Forwarded-Host",
	"X-Forwarded-Proto",
}

type wsHandshakeError struct {
	statusCode int
	body       string
	err        error
}

func (e *wsHandshakeError) Error() string {
	if e.body != "" {
		return fmt.Sprintf("WebSocket handshake failed (HTTP %d %s: %s): %v", e.statusCode, http.StatusText(e.statusCode), e.body, e.err)
	}
	return fmt.Sprintf("WebSocket handshake failed (HTTP %d %s): %v", e.statusCode, http.StatusText(e.statusCode), e.err)
}

func (e *wsHandshakeError) Unwrap() error {
	return e.err
}

func (e *wsHandshakeError) isUnsupported() bool {
	switch e.statusCode {
	case http.StatusBadRequest, http.StatusNotFound, http.StatusMethodNotAllowed, http.StatusNotImplemented:
		return true
	default:
		return false
	}
}

// wsRelay maintains a persistent WebSocket + yamux.Server tunnel to
// http-relay-server and serves incoming yamux streams via net/http's
// ReverseProxy, completely independent of the HTTP polling path.
type wsRelay struct {
	config      ClientConfig
	httpClient  *http.Client
	tokenSource oauth2.TokenSource
	handler     http.Handler

	initialReconnectDelay time.Duration
	maxReconnectDelay     time.Duration
	stableSessionDuration time.Duration
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
		Transport:     localTransport,
		FlushInterval: -1,
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

	wsTransport := http.DefaultTransport.(*http.Transport).Clone()
	wsTransport.ForceAttemptHTTP2 = false
	wsTransport.TLSNextProto = map[string]func(string, *tls.Conn) http.RoundTripper{}

	return &wsRelay{
		config:                cfg,
		httpClient:            &http.Client{Transport: wsTransport},
		tokenSource:           ts,
		handler:               tracedHandler,
		initialReconnectDelay: wsInitialReconnectDelay,
		maxReconnectDelay:     wsMaxReconnectDelay,
		stableSessionDuration: wsStableSessionDuration,
	}, nil
}

func (w *wsRelay) run(ctx context.Context) {
	bo := backoff.NewExponentialBackOff()
	bo.InitialInterval = w.initialReconnectDelay
	bo.RandomizationFactor = 0.5
	bo.Multiplier = 2
	bo.MaxInterval = w.maxReconnectDelay
	bo.MaxElapsedTime = 0
	bo.Reset()

	loggedUnsupported := false
	for {
		if ctx.Err() != nil {
			return
		}
		connectedAt, err := w.connectAndServe(ctx)
		if ctx.Err() != nil {
			return
		}

		if !connectedAt.IsZero() {
			loggedUnsupported = false
			if time.Since(connectedAt) >= w.stableSessionDuration {
				bo.Reset()
			}
		}

		delay := bo.NextBackOff()
		if err != nil {
			var hsErr *wsHandshakeError
			switch {
			case errors.As(err, &hsErr) && hsErr.isUnsupported():
				if !loggedUnsupported {
					slog.Info("Relay server does not support WebSocket tunnel, backing off",
						slog.Int("StatusCode", hsErr.statusCode),
						slog.Duration("RetryIn", delay),
						ilog.Err(err))
					loggedUnsupported = true
				} else {
					slog.Debug("Relay server WebSocket endpoint still unavailable, retrying",
						slog.Int("StatusCode", hsErr.statusCode),
						slog.Duration("RetryIn", delay))
				}
			case errors.As(err, &hsErr) && (hsErr.statusCode == http.StatusUnauthorized || hsErr.statusCode == http.StatusForbidden):
				slog.Warn("WebSocket relay authentication failed, retrying",
					slog.Int("StatusCode", hsErr.statusCode),
					slog.Duration("RetryIn", delay),
					ilog.Err(err))
			default:
				slog.Warn("WebSocket relay tunnel disconnected, retrying",
					slog.Duration("RetryIn", delay),
					ilog.Err(err))
			}
		}

		timer := time.NewTimer(delay)
		select {
		case <-ctx.Done():
			timer.Stop()
			return
		case <-timer.C:
		}
	}
}

func (w *wsRelay) connectAndServe(ctx context.Context) (time.Time, error) {
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
			return time.Time{}, fmt.Errorf("fetching relay auth token: %w", err)
		}
		header = http.Header{}
		tok.SetAuthHeader(&http.Request{Header: header})
	}

	dialCtx, cancelDial := context.WithTimeout(ctx, wsHandshakeTimeout)
	ws, resp, err := websocket.Dial(dialCtx, wsURL, &websocket.DialOptions{
		HTTPClient: w.httpClient,
		HTTPHeader: header,
	})
	cancelDial()
	if err != nil {
		if resp != nil {
			return time.Time{}, formatHandshakeError(resp, err)
		}
		return time.Time{}, err
	}

	nc := websocket.NetConn(ctx, ws, websocket.MessageBinary)
	sess, err := yamux.Server(nc, httprelay.YamuxConfig())
	if err != nil {
		_ = nc.Close()
		return time.Time{}, err
	}
	defer sess.Close()

	connectedAt := time.Now()
	slog.Info("Connected to relay server over WebSocket", slog.String("ServerName", w.config.ServerName))

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
		return connectedAt, nil
	}
	return connectedAt, err
}

func formatHandshakeError(resp *http.Response, err error) error {
	var body string
	if resp.Body != nil {
		b, _ := io.ReadAll(io.LimitReader(resp.Body, wsMaxErrorBodyBytes))
		body = strings.TrimSpace(string(b))
	}
	return &wsHandshakeError{
		statusCode: resp.StatusCode,
		body:       body,
		err:        err,
	}
}
