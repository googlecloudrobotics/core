package main

import (
	"context"
	"fmt"
	"io"
	"net"
	"net/http"
	"net/http/httptest"
	"net/http/httputil"
	"net/url"
	"strings"
	"sync"
	"testing"
	"time"

	"github.com/googlecloudrobotics/core/src/go/cmd/http-relay-client/client"
	"github.com/googlecloudrobotics/core/src/go/cmd/http-relay-server/server"
	rt "github.com/googlecloudrobotics/core/src/go/tests/relay"
	"google.golang.org/grpc"
	"google.golang.org/grpc/metadata"
)

const (
	numServerStreamedMessages = 5
)

type testServer struct {
	rt.UnimplementedTestServiceServer
}

func (s *testServer) Unary(ctx context.Context, req *rt.StringMessage) (*rt.StringMessage, error) {
	return &rt.StringMessage{Value: "Echo: " + req.Value}, nil
}

func (s *testServer) ClientStream(stream rt.TestService_ClientStreamServer) error {
	var received []string
	for {
		in, err := stream.Recv()
		if err == io.EOF {
			break
		}
		if err != nil {
			return err
		}
		received = append(received, in.Value)
	}
	respVal := fmt.Sprintf("Echo: %v", received)
	return stream.SendAndClose(&rt.StringMessage{Value: respVal})
}

func (s *testServer) ServerStream(req *rt.StringMessage, stream rt.TestService_ServerStreamServer) error {
	for i := 0; i < numServerStreamedMessages; i++ {
		respVal := fmt.Sprintf("Echo %d: %s", i, req.Value)
		if err := stream.Send(&rt.StringMessage{Value: respVal}); err != nil {
			return err
		}
	}
	return nil
}

func (s *testServer) BiDiStream(stream rt.TestService_BiDiStreamServer) error {
	for {
		in, err := stream.Recv()
		if err == io.EOF {
			return nil
		}
		if err != nil {
			return err
		}
		respVal := fmt.Sprintf("Echo: %s", in.Value)
		if err = stream.Send(&rt.StringMessage{Value: respVal}); err != nil {
			return err
		}
	}
}

func mustInitRelay(t *testing.T) (*rt.RelayEnv, net.Listener) {
	t.Helper()

	config := client.DefaultClientConfig()
	config.RelayScheme = "http"
	config.BackendScheme = "http"
	config.DisableAuthForRemote = true
	config.ForceHttp2 = true // Enable HTTP/2 for gRPC
	config.NumPendingRequests = 5

	env, lis, cleanup := rt.SetupRelay(t, config)
	t.Cleanup(cleanup)

	return env, lis
}

func TestUnaryOverRelay(t *testing.T) {
	relayEnv, lis := mustInitRelay(t)
	defer lis.Close()

	grpcServer := grpc.NewServer()
	rt.RegisterTestServiceServer(grpcServer, &testServer{})
	go func() {
		grpcServer.Serve(lis)
	}()
	defer grpcServer.Stop()

	rt.WaitForClient(t, relayEnv.RelayPort, "server_name")

	conn, err := grpc.Dial(fmt.Sprintf("127.0.0.1:%d", relayEnv.RelayPort), grpc.WithInsecure())
	if err != nil {
		t.Fatalf("Failed to create client connection: %v", err)
	}
	defer conn.Close()

	grpcClient := rt.NewTestServiceClient(conn)
	ctx := metadata.AppendToOutgoingContext(t.Context(), "x-server-name", "server_name")
	ctx, cancel := context.WithTimeout(ctx, 10*time.Second)
	defer cancel()

	reqVal := "hello"
	t.Logf("Calling Unary with %q", reqVal)
	resp, err := grpcClient.Unary(ctx, &rt.StringMessage{Value: reqVal})
	if err != nil {
		t.Fatalf("failed to call Unary: %v", err)
	}

	t.Logf("Received: %q", resp.Value)
	if want := "Echo: " + reqVal; resp.Value != want {
		t.Errorf("got %q, want %q", resp.Value, want)
	}
}

func TestServerStreamingOverRelay(t *testing.T) {
	relayEnv, lis := mustInitRelay(t)
	defer lis.Close()

	grpcServer := grpc.NewServer()
	rt.RegisterTestServiceServer(grpcServer, &testServer{})
	go func() {
		grpcServer.Serve(lis)
	}()
	defer grpcServer.Stop()

	rt.WaitForClient(t, relayEnv.RelayPort, "server_name")

	relayAddr := fmt.Sprintf("127.0.0.1:%d", relayEnv.RelayPort)
	conn, err := grpc.Dial(relayAddr, grpc.WithInsecure())
	if err != nil {
		t.Fatalf("Failed to create client connection: %v", err)
	}
	defer conn.Close()

	grpcClient := rt.NewTestServiceClient(conn)
	ctx := metadata.AppendToOutgoingContext(t.Context(), "x-server-name", "server_name")
	ctx, cancel := context.WithTimeout(ctx, 10*time.Second)
	defer cancel()

	reqVal := "hello"
	t.Logf("Calling ServerStream with %q", reqVal)
	stream, err := grpcClient.ServerStream(ctx, &rt.StringMessage{Value: reqVal})
	if err != nil {
		t.Fatalf("failed to call ServerStream: %v", err)
	}

	for i := 0; i < numServerStreamedMessages; i++ {
		resp, err := stream.Recv()
		if err != nil {
			t.Fatalf("failed to recv: %v", err)
		}
		t.Logf("Received: %q", resp.Value)
		if want := fmt.Sprintf("Echo %d: %s", i, reqVal); resp.Value != want {
			t.Errorf("got %q, want %q", resp.Value, want)
		}
	}
	if _, err = stream.Recv(); err != io.EOF {
		t.Errorf("expected EOF, got %v", err)
	}
}

func TestClientStreamingOverRelay(t *testing.T) {
	relayEnv, lis := mustInitRelay(t)
	defer lis.Close()

	grpcServer := grpc.NewServer()
	rt.RegisterTestServiceServer(grpcServer, &testServer{})
	go func() {
		grpcServer.Serve(lis)
	}()
	defer grpcServer.Stop()

	rt.WaitForClient(t, relayEnv.RelayPort, "server_name")

	relayAddr := fmt.Sprintf("127.0.0.1:%d", relayEnv.RelayPort)
	conn, err := grpc.Dial(relayAddr, grpc.WithInsecure())
	if err != nil {
		t.Fatalf("Failed to create client connection: %v", err)
	}
	defer conn.Close()

	grpcClient := rt.NewTestServiceClient(conn)
	ctx := metadata.AppendToOutgoingContext(t.Context(), "x-server-name", "server_name")
	ctx, cancel := context.WithTimeout(ctx, 10*time.Second)
	defer cancel()

	stream, err := grpcClient.ClientStream(ctx)
	if err != nil {
		t.Fatalf("failed to call ClientStream: %v", err)
	}

	messages := []string{"hello", "world", "client", "streaming"}
	for _, msg := range messages {
		t.Logf("Sending: %q", msg)
		if err := stream.Send(&rt.StringMessage{Value: msg}); err != nil {
			t.Fatalf("failed to send: %v", err)
		}
	}

	resp, err := stream.CloseAndRecv()
	if err != nil {
		t.Fatalf("failed to CloseAndRecv: %v", err)
	}

	t.Logf("Received: %q", resp.Value)
	if want := fmt.Sprintf("Echo: %v", messages); resp.Value != want {
		t.Errorf("got %q, want %q", resp.Value, want)
	}
}

func TestBiDiStreamingOverRelay(t *testing.T) {
	relayEnv, lis := mustInitRelay(t)
	defer lis.Close()

	grpcServer := grpc.NewServer()
	rt.RegisterTestServiceServer(grpcServer, &testServer{})
	go func() {
		grpcServer.Serve(lis)
	}()
	defer grpcServer.Stop()

	rt.WaitForClient(t, relayEnv.RelayPort, "server_name")

	relayAddr := fmt.Sprint("127.0.0.1:", relayEnv.RelayPort)
	conn, err := grpc.Dial(relayAddr, grpc.WithInsecure())
	if err != nil {
		t.Fatalf("Failed to create client connection: %v", err)
	}
	defer conn.Close()

	grpcClient := rt.NewTestServiceClient(conn)

	ctx := metadata.AppendToOutgoingContext(t.Context(), "x-server-name", "server_name")
	ctx, cancel := context.WithTimeout(ctx, 10*time.Second)
	defer cancel()

	stream, err := grpcClient.BiDiStream(ctx)
	if err != nil {
		t.Fatalf("failed to call BiDiStream: %v", err)
	}

	messages := []string{"hello", "world", "bidi", "streaming", "test"}

	var wg sync.WaitGroup
	wg.Go(func() {
		for _, msg := range messages {
			t.Logf("Sending: %q", msg)
			if err := stream.Send(&rt.StringMessage{Value: msg}); err != nil {
				t.Errorf("failed to send: %v", err)
				return
			}
		}
		if err := stream.CloseSend(); err != nil {
			t.Errorf("failed to CloseSend: %v", err)
		}
	})
	wg.Go(func() {
		for i, wantMsg := range messages {
			resp, err := stream.Recv()
			if err != nil {
				t.Errorf("failed to recv: %v", err)
				return
			}
			t.Logf("Received: %q", resp.Value)
			if want := fmt.Sprintf("Echo: %s", wantMsg); resp.Value != want {
				t.Errorf("got %q, want %q at index %d", resp.Value, want, i)
			}
		}
		if _, err := stream.Recv(); err != io.EOF {
			t.Errorf("expected EOF, got %v", err)
		}
	})
	wg.Wait()
}

func TestInteractiveBiDiStreamingOverRelay(t *testing.T) {
	relayEnv, lis := mustInitRelay(t)
	defer lis.Close()

	grpcServer := grpc.NewServer()
	rt.RegisterTestServiceServer(grpcServer, &testServer{})
	go func() {
		_ = grpcServer.Serve(lis)
	}()
	defer grpcServer.Stop()

	rt.WaitForClient(t, relayEnv.RelayPort, "server_name")

	relayAddr := fmt.Sprintf("127.0.0.1:%d", relayEnv.RelayPort)
	conn, err := grpc.Dial(relayAddr, grpc.WithInsecure())
	if err != nil {
		t.Fatalf("Failed to create client connection: %v", err)
	}
	defer conn.Close()

	grpcClient := rt.NewTestServiceClient(conn)
	ctx := metadata.AppendToOutgoingContext(t.Context(), "x-server-name", "server_name")
	ctx, cancel := context.WithTimeout(ctx, 10*time.Second)
	defer cancel()

	stream, err := grpcClient.BiDiStream(ctx)
	if err != nil {
		t.Fatalf("BiDiStream() failed: %v", err)
	}

	for _, msg := range []string{"hello", "world", "interactive"} {
		if err := stream.Send(&rt.StringMessage{Value: msg}); err != nil {
			t.Fatalf("stream.Send(%q) failed: %v", msg, err)
		}
		resp, err := stream.Recv()
		if err != nil {
			t.Fatalf("stream.Recv() after Send(%q) failed: %v", msg, err)
		}
		if got, want := resp.Value, "Echo: "+msg; got != want {
			t.Errorf("BiDiStream(%q) = %q, want %q", msg, got, want)
		}
	}
	if err := stream.CloseSend(); err != nil {
		t.Errorf("stream.CloseSend() failed: %v", err)
	}
	if _, err := stream.Recv(); err != io.EOF {
		t.Errorf("stream.Recv() after CloseSend = %v, want %v", err, io.EOF)
	}
}

func TestUserClientCancelReachesBackend(t *testing.T) {
	relayEnv, lis := mustInitRelay(t)
	defer lis.Close()

	backendDone := make(chan struct{})
	backend := &http.Server{
		Handler: http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			if r.URL.Path != "/wait" {
				return
			}
			w.WriteHeader(http.StatusOK)
			w.(http.Flusher).Flush()
			<-r.Context().Done()
			close(backendDone)
		}),
		Protocols: new(http.Protocols),
	}
	backend.Protocols.SetUnencryptedHTTP2(true)
	go func() {
		_ = backend.Serve(lis)
	}()
	defer backend.Close()

	rt.WaitForClient(t, relayEnv.RelayPort, "server_name")

	ctx, cancel := context.WithCancel(t.Context())
	defer cancel()
	targetURL := fmt.Sprintf("http://127.0.0.1:%d/client/server_name/wait", relayEnv.RelayPort)
	req, err := http.NewRequestWithContext(ctx, http.MethodGet, targetURL, nil)
	if err != nil {
		t.Fatalf("http.NewRequestWithContext(%q) failed: %v", targetURL, err)
	}
	resp, err := http.DefaultClient.Do(req)
	if err != nil {
		t.Fatalf("http.DefaultClient.Do(%q) failed: %v", targetURL, err)
	}
	defer resp.Body.Close()

	cancel()
	select {
	case <-backendDone:
	case <-time.After(2 * time.Second):
		t.Error("backend request context remained open 2s after user-client canceled")
	}
}

func TestFallbackToHTTPWhenGRPCBlocked(t *testing.T) {
	relayLn, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatalf("net.Listen(relay) failed: %v", err)
	}
	relayPort := relayLn.Addr().(*net.TCPAddr).Port
	relayServer := server.NewServer(server.Config{BlockSize: 10 * 1024})
	go relayServer.StartOnListener(t.Context(), relayLn)

	// Place an HTTP/1.1-only reverse proxy between relay-client and relay-server,
	// simulating an ingress that does not forward h2c gRPC on /server.
	relayTarget, err := url.Parse(fmt.Sprintf("http://127.0.0.1:%d", relayPort))
	if err != nil {
		t.Fatalf("url.Parse() failed: %v", err)
	}
	h1Gateway := httptest.NewServer(httputil.NewSingleHostReverseProxy(relayTarget))
	defer func() {
		h1Gateway.CloseClientConnections()
		h1Gateway.Close()
	}()

	backendLn, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatalf("net.Listen(backend) failed: %v", err)
	}
	defer backendLn.Close()

	grpcServer := grpc.NewServer()
	rt.RegisterTestServiceServer(grpcServer, &testServer{})
	go func() {
		_ = grpcServer.Serve(backendLn)
	}()
	defer grpcServer.Stop()

	cfg := client.DefaultClientConfig()
	cfg.RelayScheme = "http"
	cfg.RelayAddress = strings.TrimPrefix(h1Gateway.URL, "http://")
	cfg.BackendScheme = "http"
	cfg.BackendAddress = backendLn.Addr().String()
	cfg.DisableAuthForRemote = true
	cfg.ForceHttp2 = true
	cfg.NumPendingRequests = 2

	clientCtx, stopClient := context.WithCancel(t.Context())
	defer stopClient()
	relayClient := client.NewClient(cfg)
	go relayClient.Start(clientCtx)

	rt.WaitForClient(t, relayPort, "server_name")

	conn, err := grpc.Dial(fmt.Sprintf("127.0.0.1:%d", relayPort), grpc.WithInsecure())
	if err != nil {
		t.Fatalf("grpc.Dial() failed: %v", err)
	}
	defer conn.Close()

	grpcClient := rt.NewTestServiceClient(conn)
	ctx := metadata.AppendToOutgoingContext(t.Context(), "x-server-name", "server_name")
	ctx, cancel := context.WithTimeout(ctx, 10*time.Second)
	defer cancel()

	resp, err := grpcClient.Unary(ctx, &rt.StringMessage{Value: "fallback"})
	if err != nil {
		t.Fatalf("Unary(\"fallback\") over HTTP fallback failed: %v", err)
	}
	if got, want := resp.Value, "Echo: fallback"; got != want {
		t.Errorf("Unary(\"fallback\") = %q, want %q", got, want)
	}
}
