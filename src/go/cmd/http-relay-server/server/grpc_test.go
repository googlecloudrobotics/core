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
	"bufio"
	"io"
	"net"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"

	pb "github.com/googlecloudrobotics/core/src/proto/http-relay"
	"google.golang.org/grpc"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/credentials/insecure"
	"google.golang.org/grpc/status"
)

var testServeDesc = &grpc.StreamDesc{
	StreamName:    "Serve",
	ServerStreams: true,
	ClientStreams: true,
}

func TestBrokerPreferGRPCTransitions(t *testing.T) {
	s := NewServer(Config{})
	if s.b.PreferGRPC("bot") {
		t.Fatal("PreferGRPC(\"bot\") = true for unknown backend, want false")
	}

	s.b.SetGRPC("bot", true)
	if !s.b.PreferGRPC("bot") {
		t.Fatal("PreferGRPC(\"bot\") = false after SetGRPC(true), want true")
	}

	// A dual-wire client poll carrying grpc=1 must preserve gRPC preference.
	dualReq := httptest.NewRequest(http.MethodGet, "/server/request?server=bot&grpc=1", nil)
	s.serverRequest(httptest.NewRecorder(), dualReq)
	if !s.b.PreferGRPC("bot") {
		t.Error("PreferGRPC(\"bot\") = false after /server/request?grpc=1, want true")
	}

	// A legacy HTTP-only poll without grpc=1 must clear gRPC preference.
	legacyReq := httptest.NewRequest(http.MethodGet, "/server/request?server=bot", nil)
	s.serverRequest(httptest.NewRecorder(), legacyReq)
	if s.b.PreferGRPC("bot") {
		t.Error("PreferGRPC(\"bot\") = true after /server/request without grpc=1, want false")
	}
}

func TestGRPCRelayServeMissingHello(t *testing.T) {
	ln, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatalf("net.Listen() failed: %v", err)
	}
	s := NewServer(Config{})
	go s.StartOnListener(t.Context(), ln)

	cc, err := grpc.NewClient(ln.Addr().String(), grpc.WithTransportCredentials(insecure.NewCredentials()))
	if err != nil {
		t.Fatalf("grpc.NewClient() failed: %v", err)
	}
	defer cc.Close()

	stream, err := cc.NewStream(t.Context(), testServeDesc, "/server/cloudrobotics.http_relay.v1alpha1.Relay/Serve")
	if err != nil {
		t.Fatalf("cc.NewStream() failed: %v", err)
	}
	if err := stream.SendMsg(&pb.ServeRequest{}); err != nil {
		t.Fatalf("stream.SendMsg() failed: %v", err)
	}
	err = stream.RecvMsg(&pb.ServeResponse{})
	if got := status.Code(err); got != codes.InvalidArgument {
		t.Errorf("Serve(empty Hello) code = %v, want %v (err: %v)", got, codes.InvalidArgument, err)
	}
}

func TestGRPCRelayProxiesHTTPRequestOverStream(t *testing.T) {
	ln, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatalf("net.Listen() failed: %v", err)
	}
	s := NewServer(Config{})
	go s.StartOnListener(t.Context(), ln)

	cc, err := grpc.NewClient(ln.Addr().String(), grpc.WithTransportCredentials(insecure.NewCredentials()))
	if err != nil {
		t.Fatalf("grpc.NewClient() failed: %v", err)
	}
	defer cc.Close()

	stream, err := cc.NewStream(t.Context(), testServeDesc, "/server/cloudrobotics.http_relay.v1alpha1.Relay/Serve")
	if err != nil {
		t.Fatalf("cc.NewStream() failed: %v", err)
	}
	if err := stream.SendMsg(&pb.ServeRequest{
		Payload: &pb.ServeRequest_Hello{Hello: &pb.ClientHello{Server: "bot"}},
	}); err != nil {
		t.Fatalf("stream.SendMsg(Hello) failed: %v", err)
	}
	if _, err := stream.Header(); err != nil {
		t.Fatalf("stream.Header() failed: %v", err)
	}

	go func() {
		msg := &pb.ServeResponse{}
		if err := stream.RecvMsg(msg); err != nil {
			t.Errorf("stream.RecvMsg() failed: %v", err)
			return
		}
		req, err := http.ReadRequest(bufio.NewReader(strings.NewReader(string(msg.GetData()))))
		if err != nil {
			t.Errorf("http.ReadRequest() failed: %v", err)
			return
		}
		rawResp := "HTTP/1.1 202 Accepted\r\nContent-Type: text/plain\r\nContent-Length: " +
			"11\r\n\r\npath=" + req.URL.Path
		_ = stream.SendMsg(&pb.ServeRequest{
			Payload: &pb.ServeRequest_Data{Data: []byte(rawResp)},
		})
		_ = stream.CloseSend()
	}()

	resp, err := http.Get("http://" + ln.Addr().String() + "/client/bot/hello")
	if err != nil {
		t.Fatalf("http.Get(/client/bot/hello) failed: %v", err)
	}
	defer resp.Body.Close()
	body, err := io.ReadAll(resp.Body)
	if err != nil {
		t.Fatalf("io.ReadAll(resp.Body) failed: %v", err)
	}
	if resp.StatusCode != http.StatusAccepted {
		t.Errorf("GET /client/bot/hello status = %d, want %d", resp.StatusCode, http.StatusAccepted)
	}
	if got, want := string(body), "path=/hello"; got != want {
		t.Errorf("GET /client/bot/hello body = %q, want %q", got, want)
	}
}
