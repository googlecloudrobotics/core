package main

import (
	"context"
	"fmt"
	"io"
	"net"
	"net/http"
	"sync"
	"testing"
	"time"

	"github.com/googlecloudrobotics/core/src/go/cmd/http-relay-client/client"
	"github.com/googlecloudrobotics/core/src/go/cmd/http-relay-server/server"
	pb "github.com/googlecloudrobotics/core/src/go/tests/relay"
	"google.golang.org/grpc"
	"google.golang.org/grpc/metadata"
)

type testServer struct {
	pb.UnimplementedTestServiceServer
}

func (s *testServer) Unary(ctx context.Context, req *pb.StringMessage) (*pb.StringMessage, error) {
	return &pb.StringMessage{Value: "Echo: " + req.Value}, nil
}

func (s *testServer) ClientStream(stream pb.TestService_ClientStreamServer) error {
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
	return stream.SendAndClose(&pb.StringMessage{Value: respVal})
}

func (s *testServer) ServerStream(req *pb.StringMessage, stream pb.TestService_ServerStreamServer) error {
	for i := 0; i < 5; i++ {
		respVal := fmt.Sprintf("Echo %d: %s", i, req.Value)
		if err := stream.Send(&pb.StringMessage{Value: respVal}); err != nil {
			return err
		}
	}
	return nil
}

func (s *testServer) BiDiStream(stream pb.TestService_BiDiStreamServer) error {
	for {
		in, err := stream.Recv()
		if err == io.EOF {
			return nil
		}
		if err != nil {
			return err
		}
		respVal := fmt.Sprintf("Echo: %s", in.Value)
		if err = stream.Send(&pb.StringMessage{Value: respVal}); err != nil {
			return err
		}
	}
}

type relayEnv struct {
	relayPort   int
	backendPort int
}

func pickUnusedPort(t *testing.T) int {
	t.Helper()
	var addr *net.TCPAddr
	var err error
	if addr, err = net.ResolveTCPAddr("tcp", "localhost:0"); err == nil {
		var list *net.TCPListener
		if list, err = net.ListenTCP("tcp", addr); err == nil {
			defer list.Close()
			return list.Addr().(*net.TCPAddr).Port
		}
	}
	t.Fatalf("Failed to pick a free TCP port: %v", err)
	return 0
}

func mustInitRelay(t *testing.T) *relayEnv {
	t.Helper()

	backendPort := pickUnusedPort(t)
	relayPort := pickUnusedPort(t)

	go func() {
		relayServer := server.NewServer(server.Config{
			Port:      relayPort,
			BlockSize: 10 * 1024,
		})
		relayServer.Start()
	}()

	go func() {
		config := client.DefaultClientConfig()
		config.RelayScheme = "http"
		config.RelayAddress = fmt.Sprint("localhost:", relayPort)
		config.BackendScheme = "http"
		config.BackendAddress = fmt.Sprint("localhost:", backendPort)
		config.DisableAuthForRemote = true
		config.ForceHttp2 = true // Enable HTTP/2 for gRPC
		config.NumPendingRequests = 5
		relayClient := client.NewClient(config)
		relayClient.Start()
	}()

	relayHealthy := false
	deadline := time.Now().Add(5 * time.Second)
	for time.Now().Before(deadline) {
		relayHealthzAddr := fmt.Sprint("http://localhost:", relayPort, "/healthz")
		res, err := http.Get(relayHealthzAddr)
		if err != nil {
			time.Sleep(250 * time.Millisecond)
		} else {
			relayHealthy = true
			res.Body.Close()
			break
		}
	}
	if !relayHealthy {
		t.Fatal("Failed to bring up http relay.")
	}

	return &relayEnv{
		relayPort:   relayPort,
		backendPort: backendPort,
	}
}

func TestUnaryOverRelay(t *testing.T) {
	relayEnv := mustInitRelay(t)

	lis, err := net.Listen("tcp", fmt.Sprintf("localhost:%d", relayEnv.backendPort))
	if err != nil {
		t.Fatalf("failed to listen on backend port %d: %v", relayEnv.backendPort, err)
	}
	grpcServer := grpc.NewServer()
	pb.RegisterTestServiceServer(grpcServer, &testServer{})
	go func() {
		if err := grpcServer.Serve(lis); err != nil {
			t.Fatalf("grpcServer.Serve(lis) failed unexpectedly: %v", err)
		}
	}()
	defer grpcServer.Stop()

	relayAddr := fmt.Sprintf("localhost:%d", relayEnv.relayPort)
	conn, err := grpc.Dial(relayAddr, grpc.WithInsecure())
	if err != nil {
		t.Fatalf("Failed to create client connection: %v", err)
	}
	defer conn.Close()

	grpcClient := pb.NewTestServiceClient(conn)
	ctx := metadata.AppendToOutgoingContext(context.Background(), "x-server-name", "server_name")
	ctx, cancel := context.WithTimeout(ctx, 10*time.Second)
	defer cancel()

	reqVal := "hello"
	t.Logf("Calling Unary with %q", reqVal)
	resp, err := grpcClient.Unary(ctx, &pb.StringMessage{Value: reqVal})
	if err != nil {
		t.Fatalf("failed to call Unary: %v", err)
	}

	t.Logf("Received: %q", resp.Value)
	if want := "Echo: " + reqVal; resp.Value != want {
		t.Errorf("got %q, want %q", resp.Value, want)
	}
}

func TestServerStreamingOverRelay(t *testing.T) {
	relayEnv := mustInitRelay(t)

	lis, err := net.Listen("tcp", fmt.Sprintf("localhost:%d", relayEnv.backendPort))
	if err != nil {
		t.Fatalf("failed to listen on backend port %d: %v", relayEnv.backendPort, err)
	}
	grpcServer := grpc.NewServer()
	pb.RegisterTestServiceServer(grpcServer, &testServer{})
	go func() {
		if err := grpcServer.Serve(lis); err != nil {
			t.Fatalf("grpcServer.Serve(lis) failed unexpectedly: %v", err)
		}
	}()
	defer grpcServer.Stop()

	relayAddr := fmt.Sprintf("localhost:%d", relayEnv.relayPort)
	conn, err := grpc.Dial(relayAddr, grpc.WithInsecure())
	if err != nil {
		t.Fatalf("Failed to create client connection: %v", err)
	}
	defer conn.Close()

	grpcClient := pb.NewTestServiceClient(conn)
	ctx := metadata.AppendToOutgoingContext(context.Background(), "x-server-name", "server_name")
	ctx, cancel := context.WithTimeout(ctx, 10*time.Second)
	defer cancel()

	reqVal := "hello"
	t.Logf("Calling ServerStream with %q", reqVal)
	stream, err := grpcClient.ServerStream(ctx, &pb.StringMessage{Value: reqVal})
	if err != nil {
		t.Fatalf("failed to call ServerStream: %v", err)
	}

	for i := 0; i < 5; i++ {
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
	relayEnv := mustInitRelay(t)

	lis, err := net.Listen("tcp", fmt.Sprintf("localhost:%d", relayEnv.backendPort))
	if err != nil {
		t.Fatalf("failed to listen on backend port %d: %v", relayEnv.backendPort, err)
	}
	grpcServer := grpc.NewServer()
	pb.RegisterTestServiceServer(grpcServer, &testServer{})
	go func() {
		if err := grpcServer.Serve(lis); err != nil {
			t.Fatalf("grpcServer.Serve(lis) failed unexpectedly: %v", err)
		}
	}()
	defer grpcServer.Stop()

	relayAddr := fmt.Sprintf("localhost:%d", relayEnv.relayPort)
	conn, err := grpc.Dial(relayAddr, grpc.WithInsecure())
	if err != nil {
		t.Fatalf("Failed to create client connection: %v", err)
	}
	defer conn.Close()

	grpcClient := pb.NewTestServiceClient(conn)
	ctx := metadata.AppendToOutgoingContext(context.Background(), "x-server-name", "server_name")
	ctx, cancel := context.WithTimeout(ctx, 10*time.Second)
	defer cancel()

	stream, err := grpcClient.ClientStream(ctx)
	if err != nil {
		t.Fatalf("failed to call ClientStream: %v", err)
	}

	messages := []string{"hello", "world", "client", "streaming"}
	for _, msg := range messages {
		t.Logf("Sending: %q", msg)
		if err := stream.Send(&pb.StringMessage{Value: msg}); err != nil {
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
	relayEnv := mustInitRelay(t)

	lis, err := net.Listen("tcp", fmt.Sprintf("localhost:%d", relayEnv.backendPort))
	if err != nil {
		t.Fatalf("failed to listen on backend port %d: %v", relayEnv.backendPort, err)
	}
	grpcServer := grpc.NewServer()
	pb.RegisterTestServiceServer(grpcServer, &testServer{})
	go func() {
		if err := grpcServer.Serve(lis); err != nil {
			t.Fatalf("grpcServer.Serve(lis) failed unexpectedly: %v", err)
		}
	}()
	defer grpcServer.Stop()

	relayAddr := fmt.Sprint("localhost:", relayEnv.relayPort)
	conn, err := grpc.Dial(relayAddr, grpc.WithInsecure())
	if err != nil {
		t.Fatalf("Failed to create client connection: %v", err)
	}
	defer conn.Close()

	grpcClient := pb.NewTestServiceClient(conn)

	ctx := metadata.AppendToOutgoingContext(context.Background(), "x-server-name", "server_name")
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
			if err := stream.Send(&pb.StringMessage{Value: msg}); err != nil {
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
