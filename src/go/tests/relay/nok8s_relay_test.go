// Copyright 2022 The Cloud Robotics Authors
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

package main

import (
	"bytes"
	"context"
	"fmt"
	"io"
	"log"
	"net"
	"net/http"
	"net/http/httptest"
	"os"
	"os/exec"
	"regexp"
	"strings"
	"testing"
	"time"

	"github.com/pkg/errors"
	"google.golang.org/grpc"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/metadata"
	"google.golang.org/grpc/status"
	testpb "google.golang.org/grpc/test/grpc_testing"
)

const (
	RelayClientPath = "src/go/cmd/http-relay-client/http-relay-client-bin_/http-relay-client-bin"
	RelayServerPath = "src/go/cmd/http-relay-server/http-relay-server-bin_/http-relay-server-bin"
)

var (
	RelayClientArgs = []string{
		"--backend_scheme=http",
		"--relay_scheme=http",
		"--server_name=remote1",
		"--disable_auth_for_remote",
	}
	RelayServerArgs = []string{
		"--port=0",
	}
	rsPortMatcher = regexp.MustCompile(`Relay server listening on: 127.0.0.1:(\d\d*)\n$`)
)

type relay struct {
	rs, rc *exec.Cmd
	rsPort string
}

// start brings up the relay processes
func (r *relay) start(backendAddress string, extraClientArgs ...string) error {
	// run relay server exposing the relay client
	var rsOut bytes.Buffer
	r.rs = exec.Command(RelayServerPath, RelayServerArgs...)
	r.rs.Stdout = os.Stdout
	r.rs.Stderr = io.MultiWriter(os.Stderr, &rsOut)
	if err := r.rs.Start(); err != nil {
		return errors.Wrap(err, "failed to start relay-server")
	}
	r.rsPort = ""
	for i := 0; i < 10; i++ {
		if m := rsPortMatcher.FindStringSubmatch(rsOut.String()); m != nil {
			r.rsPort = m[1]
			log.Printf("Server port: %s", r.rsPort)
			break
		}
		log.Print("Waiting for relay to be up-and-running ...")
		time.Sleep(1 * time.Second)
	}
	if r.rsPort == "" {
		return errors.New("timeout waiting for relay-server to launch")
	}

	// run relay client exposing the test-backend
	rcArgs := append(RelayClientArgs, []string{
		"--backend_address=" + backendAddress,
		"--relay_address=127.0.0.1:" + r.rsPort,
	}...)
	rcArgs = append(rcArgs, extraClientArgs...)
	log.Printf("Backend address: %s", backendAddress)

	r.rc = exec.Command(RelayClientPath, rcArgs...)
	r.rc.Stdout = os.Stdout
	r.rc.Stderr = os.Stderr
	if err := r.rc.Start(); err != nil {
		return errors.Wrap(err, "failed to start relay-client")
	}

	connected := false
	for i := 0; i < 10; i++ {
		if strings.Contains(rsOut.String(), "Relay client connected") {
			connected = true
			break
		}
		log.Print("Waiting for relay to be up-and-running ...")
		time.Sleep(1 * time.Second)
	}
	if !connected {
		errors.New("timeout waiting for relay-client to connect to relay-server")
	}
	return nil
}

// stop tears down the relay processes
func (r *relay) stop() error {
	if err := r.rs.Process.Kill(); err != nil {
		return errors.Wrap(err, "failed to kill relay-server")
	}
	if err := r.rc.Process.Kill(); err != nil {
		return errors.Wrap(err, "failed to kill relay-client")
	}
	return nil
}

// TestHttpRelay launches a local http relay (client + server) and connects a
// test-hhtp-server as a backend. The test is then interacting with the backend
// through the local relay.
func TestHttpRelay(t *testing.T) {
	tests := []struct {
		desc       string
		urlPath    string
		statusCode int
		body       string
	}{
		{
			desc:       "simple get",
			urlPath:    "/client/remote1/",
			statusCode: http.StatusOK,
			body:       "Hello",
		},
		{
			desc:       "backend status is preserved",
			urlPath:    "/client/remote1/bad-path",
			statusCode: http.StatusNotFound,
			body:       "",
		},
		{
			desc:       "invalid client",
			urlPath:    "/client/wrong/",
			statusCode: http.StatusInternalServerError,
			body:       "doesn't appear to be running the relay client",
		},
	}

	// setup http test server
	ts := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path == "/" {
			fmt.Fprintln(w, "Hello")
			return
		}
		w.WriteHeader(http.StatusNotFound)
	}))
	defer ts.Close()

	backendAddress := strings.TrimPrefix(ts.URL, "http://")
	r := &relay{}
	if err := r.start(backendAddress); err != nil {
		t.Fatal("failed to start relay: ", err)
	}
	defer func() {
		if err := r.stop(); err != nil {
			t.Fatal("failed to stop relay: ", err)
		}
	}()
	relayAddress := "http://127.0.0.1:" + r.rsPort

	for _, tc := range tests {
		t.Run(tc.desc, func(t *testing.T) {
			res, err := http.Get(relayAddress + tc.urlPath)
			if err != nil {
				t.Fatal(err)
			}
			body, err := io.ReadAll(res.Body)
			res.Body.Close()
			if err != nil {
				t.Fatal(err)
			}

			if res.StatusCode != tc.statusCode {
				t.Errorf("Wrong status code - got %d, expected %d", res.StatusCode, tc.statusCode)
			}
			if !strings.Contains(string(body), tc.body) {
				t.Errorf("Wrong body - got %q, expected it to contain %q, ", body, tc.body)
			}
		})
	}
}

type testServer struct {
	testpb.UnimplementedTestServiceServer
}

func (s *testServer) EmptyCall(ctx context.Context, in *testpb.Empty) (*testpb.Empty, error) {
	return &testpb.Empty{}, nil
}

// TestGrpcRelaySimpleCallWorks launches a local http relay (client + server), connects a
// grpc service as backend and issues a simple call.
func TestGrpcRelaySimpleCallWorks(t *testing.T) {
	// setup grpc test server
	l, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Errorf("failed to listen: %v", err)
	}
	defer l.Close()
	s := grpc.NewServer()
	testpb.RegisterTestServiceServer(s, &testServer{})
	go s.Serve(l)
	defer s.Stop()

	backendAddress := fmt.Sprintf("127.0.0.1:%d", l.Addr().(*net.TCPAddr).Port)
	r := &relay{}
	if err := r.start(backendAddress, "--force_http2"); err != nil {
		t.Fatal("failed to start relay: ", err)
	}
	defer func() {
		if err := r.stop(); err != nil {
			t.Fatal("failed to stop relay: ", err)
		}
	}()
	relayAddress := "127.0.0.1:" + r.rsPort

	ctx := metadata.AppendToOutgoingContext(context.Background(), "x-server-name", "remote1")
	conn, err := grpc.DialContext(ctx, relayAddress, grpc.WithInsecure())
	if err != nil {
		t.Errorf("Failed to create client connection: %v", err)
	}
	defer conn.Close()

	client := testpb.NewTestServiceClient(conn)

	if _, err = client.EmptyCall(ctx, &testpb.Empty{}); err != nil {
		if ec, ok := status.FromError(err); ok {
			if ec.Code() != codes.OK {
				t.Errorf("Wrong error code: got %d, expected %d", ec.Code(), codes.OK)
			}
		}
	}
}

// TestGrpcRelayErrorArePropagated launches a local http relay (client + server), connects a
// grpc service as backend and issues a simple call.
func TestGrpcRelayErrorArePropagated(t *testing.T) {
	// setup grpc test server
	l, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Errorf("failed to listen: %v", err)
	}
	defer l.Close()
	s := grpc.NewServer()
	testpb.RegisterTestServiceServer(s, &testServer{})
	go s.Serve(l)
	defer s.Stop()

	backendAddress := fmt.Sprintf("127.0.0.1:%d", l.Addr().(*net.TCPAddr).Port)
	r := &relay{}
	if err := r.start(backendAddress, "--force_http2"); err != nil {
		t.Fatal("failed to start relay: ", err)
	}
	defer func() {
		if err := r.stop(); err != nil {
			t.Fatal("failed to stop relay: ", err)
		}
	}()
	relayAddress := "127.0.0.1:" + r.rsPort

	ctx := metadata.AppendToOutgoingContext(context.Background(), "x-server-name", "remote1")
	conn, err := grpc.DialContext(ctx, relayAddress, grpc.WithInsecure())
	if err != nil {
		t.Errorf("Failed to create client connection: %v", err)
	}
	defer conn.Close()

	client := testpb.NewTestServiceClient(conn)

	if _, err = client.UnaryCall(ctx, &testpb.SimpleRequest{}); err != nil {
		if ec, ok := status.FromError(err); ok {
			if ec.Code() != codes.Unimplemented {
				t.Errorf("Wrong error code: got %d, expected %d", ec.Code(), codes.Unimplemented)
			}
		}
	}
}
