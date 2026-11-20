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
	"sync/atomic"
	"time"

	"github.com/googlecloudrobotics/core/src/go/pkg/httprelay"
	pb "github.com/googlecloudrobotics/core/src/proto/http-relay"
	"github.com/googlecloudrobotics/ilog"
	"google.golang.org/grpc"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/metadata"
	"google.golang.org/grpc/status"
)

const (
	// pathPrefix is the prefix under which ingresses route relay-client traffic
	// to http-relay-server.
	pathPrefix = "/server"
	// queryParam indicates on /server/request that the relay client also
	// maintains gRPC streams, so polling should not clear backend gRPC state.
	queryParam = "grpc"
	// keepAliveInterval is how often an empty message is sent on an active
	// stream so intermediate proxies do not time out quiet connections.
	keepAliveInterval = 30 * time.Second

	initialWindowSize     = 16 << 20
	initialConnWindowSize = 32 << 20
	proxyReadBufferSize   = 64 * 1024
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
	state  *requestStreamState
}

type serverStream struct {
	conn     *httprelay.StreamConn
	done     chan struct{}
	doneOnce sync.Once
	aborted  atomic.Bool
}

func (s *serverStream) finish() {
	s.conn.Close()
	s.doneOnce.Do(func() {
		close(s.done)
	})
}

func (s *serverStream) abort() {
	s.aborted.Store(true)
	s.finish()
}

type requestStreamState struct {
	mu      sync.Mutex
	aborted bool
	ss      *serverStream
}

func (st *requestStreamState) attach(ss *serverStream) bool {
	st.mu.Lock()
	defer st.mu.Unlock()
	if st.aborted {
		ss.abort()
		return false
	}
	st.ss = ss
	return true
}

func (st *requestStreamState) abort() {
	st.mu.Lock()
	st.aborted = true
	ss := st.ss
	st.mu.Unlock()
	if ss != nil {
		ss.abort()
	}
}

func (st *requestStreamState) finish() {
	st.mu.Lock()
	ss := st.ss
	st.mu.Unlock()
	if ss != nil {
		ss.finish()
	}
}

// gRPCRelay serves Relay.Serve streams and proxies user-client HTTP requests
// directly over claimed streams via httputil.ReverseProxy.
type gRPCRelay struct {
	pb.UnimplementedRelayServer
	b     *broker
	proxy *httputil.ReverseProxy

	mu      sync.Mutex
	waiters map[string]chan chan *serverStream
}

func newGRPCRelay(b *broker) *gRPCRelay {
	g := &gRPCRelay{
		b:       b,
		waiters: make(map[string]chan chan *serverStream),
	}
	transport := &http.Transport{
		DialContext:       g.dialStream,
		DisableKeepAlives: true,
		ReadBufferSize:    proxyReadBufferSize,
	}
	g.proxy = &httputil.ReverseProxy{
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
		ErrorHandler: func(w http.ResponseWriter, _ *http.Request, err error) {
			if errors.Is(err, context.Canceled) {
				return
			}
			slog.Error("gRPC relay proxy error", ilog.Err(err))
			if errors.Is(err, ErrRelayTimeout) {
				http.Error(w, err.Error(), http.StatusInternalServerError)
				return
			}
			http.Error(w, err.Error(), http.StatusBadGateway)
		},
	}
	return g
}

func (g *gRPCRelay) newGRPCServer() *grpc.Server {
	gs := grpc.NewServer(
		grpc.InitialWindowSize(initialWindowSize),
		grpc.InitialConnWindowSize(initialConnWindowSize),
	)
	pb.RegisterRelayServer(gs, g)
	return gs
}

func (g *gRPCRelay) waiterChan(server string) chan chan *serverStream {
	g.mu.Lock()
	defer g.mu.Unlock()
	ch := g.waiters[server]
	if ch == nil {
		ch = make(chan chan *serverStream)
		g.waiters[server] = ch
	}
	return ch
}

func (g *gRPCRelay) dialStream(ctx context.Context, _, _ string) (net.Conn, error) {
	target, ok := ctx.Value(targetKey{}).(proxyTarget)
	if !ok {
		return nil, errors.New("missing proxy target in context")
	}
	ch := g.waiterChan(target.server)
	reply := make(chan *serverStream, 1)
	select {
	case ch <- reply:
		ss := <-reply
		if !target.state.attach(ss) {
			return nil, ctx.Err()
		}
		g.b.TouchBackend(target.server)
		return ss.conn, nil
	case <-time.After(relayTimeout):
		return nil, fmt.Errorf("cannot reach the client %q; check that it's turned on, set up, and connected to the internet; if the network config recently changed, try again in 1-2 minutes (%w)", target.server, ErrRelayTimeout)
	case <-ctx.Done():
		return nil, ctx.Err()
	}
}

func (g *gRPCRelay) serveHTTP(ctx context.Context, w http.ResponseWriter, r *http.Request, backendCtx backendContext) {
	_ = http.NewResponseController(w).EnableFullDuplex()

	state := &requestStreamState{}
	stopAbort := context.AfterFunc(ctx, state.abort)
	defer func() {
		stopAbort()
		if ctx.Err() != nil {
			state.abort()
			return
		}
		state.finish()
	}()

	reqCtx := context.WithValue(ctx, targetKey{}, proxyTarget{
		server: backendCtx.ServerName,
		path:   backendCtx.Path,
		state:  state,
	})
	g.proxy.ServeHTTP(w, r.WithContext(reqCtx))
}

func (g *gRPCRelay) Serve(stream pb.Relay_ServeServer) error {
	first, err := stream.Recv()
	if err != nil {
		return err
	}
	server := first.GetHello().GetServer()
	if server == "" {
		return status.Error(codes.InvalidArgument, "missing server in ClientHello")
	}

	g.b.SetGRPC(server, true)
	if err := stream.SendHeader(metadata.MD{}); err != nil {
		return err
	}

	ch := g.waiterChan(server)
	var reply chan *serverStream
	select {
	case reply = <-ch:
	case <-time.After(getRequestTimeout):
		return status.Error(codes.DeadlineExceeded, ErrNoRequest.Error())
	case <-stream.Context().Done():
		return stream.Context().Err()
	}

	ss := &serverStream{
		done: make(chan struct{}),
		conn: httprelay.NewStreamConn(nil, httprelay.StreamFuncs{
			Send: func(chunk []byte) error {
				return stream.Send(&pb.ServeResponse{Data: chunk})
			},
			Recv: func() ([]byte, error) {
				msg, err := stream.Recv()
				if err != nil {
					return nil, err
				}
				if msg.GetHello() != nil {
					return nil, errors.New("unexpected duplicate ClientHello")
				}
				return msg.GetData(), nil
			},
		}),
	}
	reply <- ss

	ticker := time.NewTicker(keepAliveInterval)
	defer ticker.Stop()
	for {
		select {
		case <-ticker.C:
			_ = ss.conn.SendKeepAlive()
		case <-ss.done:
			if ss.aborted.Load() {
				return status.Error(codes.Canceled, "user request canceled")
			}
			return nil
		case <-stream.Context().Done():
			ss.abort()
			return stream.Context().Err()
		}
	}
}
