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
	"log/slog"
	"net"
	"net/http"
	"net/http/httputil"
	"os"
	"sync"
	"sync/atomic"
	"time"

	"github.com/googlecloudrobotics/core/src/go/pkg/httprelay"
	"github.com/googlecloudrobotics/core/src/go/pkg/telemetry"
	pb "github.com/googlecloudrobotics/core/src/proto/http-relay"
	"github.com/googlecloudrobotics/ilog"
	"go.opentelemetry.io/contrib/instrumentation/net/http/otelhttp"
	"golang.org/x/oauth2/google"
	"google.golang.org/grpc"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/credentials"
	"google.golang.org/grpc/credentials/insecure"
	"google.golang.org/grpc/credentials/oauth"
	"google.golang.org/grpc/keepalive"
	"google.golang.org/grpc/status"
)

const (
	// serveMethod is the path under which http-relay-server exposes Relay.Serve.
	// A manual StreamDesc is used so RelayPrefix + "/server" can be prepended.
	serveMethod = "/server/cloudrobotics.http_relay.v1alpha1.Relay/Serve"

	queryParam            = "grpc"
	initialWindowSize     = 16 << 20
	initialConnWindowSize = 32 << 20
	streamCloseDrain      = 5 * time.Second
)

var (
	serveDesc = &grpc.StreamDesc{
		StreamName:    "Serve",
		ClientStreams: true,
		ServerStreams: true,
	}
	preservedForwardHeaders = []string{
		"Forwarded",
		"X-Forwarded-For",
		"X-Forwarded-Host",
		"X-Forwarded-Proto",
	}
)

// gRPCRelay maintains Relay.Serve streams to http-relay-server and forwards
// incoming HTTP/upgraded connections directly to the configured backend via
// net/http's ReverseProxy, completely independent of the HTTP polling path.
type gRPCRelay struct {
	config ClientConfig
	conn   *grpc.ClientConn
	ln     *chanListener
	srv    *http.Server

	healthy   atomic.Bool
	ready     chan struct{}
	readyOnce sync.Once
}

func newGRPCRelay(ctx context.Context, cfg ClientConfig, localTransport http.RoundTripper) (*gRPCRelay, error) {
	cc, err := dialRelayServer(ctx, cfg)
	if err != nil {
		return nil, err
	}

	ln := newChanListener()
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

	return &gRPCRelay{
		config: cfg,
		conn:   cc,
		ln:     ln,
		srv: &http.Server{
			Handler: tracedHandler,
			BaseContext: func(net.Listener) context.Context {
				return ctx
			},
		},
		ready: make(chan struct{}),
	}, nil
}

func dialRelayServer(ctx context.Context, cfg ClientConfig) (*grpc.ClientConn, error) {
	opts := []grpc.DialOption{
		grpc.WithKeepaliveParams(keepalive.ClientParameters{
			Time:                cfg.ReadIdleTimeout,
			PermitWithoutStream: true,
		}),
		grpc.WithInitialWindowSize(initialWindowSize),
		grpc.WithInitialConnWindowSize(initialConnWindowSize),
	}
	if cfg.RelayScheme == "https" {
		opts = append(opts, grpc.WithTransportCredentials(credentials.NewTLS(&tls.Config{})))
		if !cfg.DisableAuthForRemote {
			const scope = "https://www.googleapis.com/auth/cloud-platform.read-only"
			ts, err := google.DefaultTokenSource(ctx, scope)
			if err != nil {
				return nil, err
			}
			opts = append(opts, grpc.WithPerRPCCredentials(oauth.TokenSource{TokenSource: ts}))
		}
	} else {
		opts = append(opts, grpc.WithTransportCredentials(insecure.NewCredentials()))
	}
	return grpc.NewClient(cfg.RelayAddress, opts...)
}

func (g *gRPCRelay) markReady() {
	g.readyOnce.Do(func() {
		close(g.ready)
	})
}

// waitReady waits until the first Relay.Serve stream handshake either succeeds
// or fails, avoiding a startup race where an HTTP poll registers first.
func (g *gRPCRelay) waitReady(ctx context.Context) {
	select {
	case <-g.ready:
	case <-ctx.Done():
	}
}

func (g *gRPCRelay) isHealthy() bool {
	return g.healthy.Load()
}

func (g *gRPCRelay) run(ctx context.Context) {
	var wg sync.WaitGroup
	wg.Go(func() {
		_ = g.srv.Serve(g.ln)
	})
	for range g.config.NumPendingRequests {
		wg.Go(func() {
			g.worker(ctx)
		})
	}

	<-ctx.Done()
	_ = g.ln.Close()
	shutdownCtx, cancel := context.WithTimeout(context.Background(), streamCloseDrain)
	_ = g.srv.Shutdown(shutdownCtx)
	cancel()
	_ = g.conn.Close()
	wg.Wait()
}

func (g *gRPCRelay) worker(ctx context.Context) {
	for {
		if ctx.Err() != nil {
			return
		}
		err := g.openAndAccept(ctx)
		if err == nil || status.Code(err) == codes.DeadlineExceeded || errors.Is(err, context.Canceled) {
			continue
		}
		slog.Debug("gRPC relay stream unavailable, retrying", ilog.Err(err))
		select {
		case <-ctx.Done():
			return
		case <-time.After(time.Second):
		}
	}
}

func (g *gRPCRelay) openAndAccept(ctx context.Context) error {
	streamCtx, cancel := context.WithCancel(ctx)
	stream, err := g.conn.NewStream(streamCtx, serveDesc, g.config.RelayPrefix+serveMethod)
	if err != nil {
		cancel()
		g.healthy.Store(false)
		g.markReady()
		return err
	}
	hello := &pb.ServeRequest{
		Payload: &pb.ServeRequest_Hello{
			Hello: &pb.ClientHello{Server: g.config.ServerName},
		},
	}
	if err := stream.SendMsg(hello); err != nil {
		cancel()
		g.healthy.Store(false)
		g.markReady()
		return err
	}
	if _, err := stream.Header(); err != nil {
		cancel()
		g.healthy.Store(false)
		g.markReady()
		return err
	}

	g.healthy.Store(true)
	g.markReady()

	var initial []byte
	for len(initial) == 0 {
		msg := &pb.ServeResponse{}
		if err := stream.RecvMsg(msg); err != nil {
			cancel()
			if status.Code(err) != codes.DeadlineExceeded && !errors.Is(err, context.Canceled) {
				g.healthy.Store(false)
			}
			return err
		}
		initial = msg.GetData()
	}

	var timerMu sync.Mutex
	var drainTimer *time.Timer
	stopStream := func() {
		timerMu.Lock()
		if drainTimer != nil {
			drainTimer.Stop()
		}
		timerMu.Unlock()
		cancel()
	}

	sc := httprelay.NewStreamConn(initial, httprelay.StreamFuncs{
		Send: func(chunk []byte) error {
			return stream.SendMsg(&pb.ServeRequest{
				Payload: &pb.ServeRequest_Data{Data: chunk},
			})
		},
		Recv: func() ([]byte, error) {
			msg := &pb.ServeResponse{}
			if err := stream.RecvMsg(msg); err != nil {
				stopStream()
				return nil, err
			}
			return msg.GetData(), nil
		},
		CloseWrite: stream.CloseSend,
		Close: func() error {
			// CloseWrite already sent END_STREAM after all response frames.
			// Give the server a short window to read EOF and close the stream
			// before canceling streamCtx.
			timerMu.Lock()
			drainTimer = time.AfterFunc(streamCloseDrain, cancel)
			timerMu.Unlock()
			return nil
		},
	})
	g.ln.enqueue(ctx, sc)
	return nil
}

type chanListener struct {
	conns     chan net.Conn
	closed    chan struct{}
	closeOnce sync.Once
}

func newChanListener() *chanListener {
	return &chanListener{
		conns:  make(chan net.Conn),
		closed: make(chan struct{}),
	}
}

func (l *chanListener) enqueue(ctx context.Context, conn net.Conn) {
	select {
	case l.conns <- conn:
	case <-l.closed:
		_ = conn.Close()
	case <-ctx.Done():
		_ = conn.Close()
	}
}

func (l *chanListener) Accept() (net.Conn, error) {
	select {
	case conn := <-l.conns:
		return conn, nil
	case <-l.closed:
		return nil, net.ErrClosed
	}
}

func (l *chanListener) Close() error {
	l.closeOnce.Do(func() {
		close(l.closed)
	})
	return nil
}

func (l *chanListener) Addr() net.Addr { return listenerAddr{} }

type listenerAddr struct{}

func (listenerAddr) Network() string { return "grpc-relay" }
func (listenerAddr) String() string  { return "grpc-relay" }
