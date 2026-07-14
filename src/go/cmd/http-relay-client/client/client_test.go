// Copyright 2023 The Cloud Robotics Authors
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
	"bytes"
	"fmt"
	"io"
	"net/http"
	"strings"
	"testing"
	"testing/synctest"
	"time"

	pb "github.com/googlecloudrobotics/core/src/proto/http-relay"

	"google.golang.org/protobuf/proto"
)

type mockTransport struct {
	roundTrip func(*http.Request) (*http.Response, error)
}

func (m *mockTransport) RoundTrip(req *http.Request) (*http.Response, error) {
	return m.roundTrip(req)
}

func TestLocalProxy(t *testing.T) {
	synctest.Test(t, func(t *testing.T) {
		// We expect the response below to always contain 0 milliseconds.
		timeSince = func(t time.Time) time.Duration { return 0 * time.Millisecond }

		reqPayload, _ := proto.Marshal(&pb.HttpRequest{
			Id:     proto.String("15"),
			Method: proto.String("GET"),
			Url:    proto.String("http://invalid/foo/bar?a=b"),
			Header: []*pb.HttpHeader{{
				Name:  proto.String("X-GFE"),
				Value: proto.String("google.com")}},
			Body: []byte("thebody"),
		})
		respPayload, _ := proto.Marshal(&pb.HttpResponse{
			Id:         proto.String("15"),
			StatusCode: proto.Int32(201),
			Header: []*pb.HttpHeader{
				{
					Name:  proto.String("Priority"),
					Value: proto.String("High"),
				},
			},
			Body:              []byte("theresponsebody"),
			Eof:               proto.Bool(true),
			BackendDurationMs: proto.Int64(0),
		})

		relayRequests := 0
		relayTransport := &mockTransport{
			roundTrip: func(req *http.Request) (*http.Response, error) {
				relayRequests++
				switch relayRequests {
				case 1:
					if req.Method != "GET" || req.URL.Path != "/server/request" {
						t.Errorf("Relay Req 1: expected GET /server/request, got %s %s", req.Method, req.URL.Path)
					}
					if req.URL.Query().Get("server") != "foo" {
						t.Errorf("Relay Req 1: expected query server=foo, got %q", req.URL.RawQuery)
					}
					return &http.Response{
						StatusCode: 200,
						Body:       io.NopCloser(bytes.NewReader(reqPayload)),
					}, nil
				case 2:
					if req.Method != "POST" || req.URL.Path != "/server/response" {
						t.Errorf("Relay Req 2: expected POST /server/response, got %s %s", req.Method, req.URL.Path)
					}
					body, _ := io.ReadAll(req.Body)
					if !bytes.Equal(body, respPayload) {
						t.Errorf("Relay Req 2: body mismatch")
					}
					return &http.Response{
						StatusCode: 200,
						Body:       io.NopCloser(strings.NewReader("")),
					}, nil
				default:
					t.Errorf("Unexpected extra request to relay: %s %s", req.Method, req.URL.Path)
					return nil, fmt.Errorf("unexpected request")
				}
			},
		}

		backendRequests := 0
		backendTransport := &mockTransport{
			roundTrip: func(req *http.Request) (*http.Response, error) {
				backendRequests++
				if req.Method != "GET" || req.URL.Path != "/foo/bar" {
					t.Errorf("Backend Req: expected GET /foo/bar, got %s %s", req.Method, req.URL.Path)
				}
				if req.URL.Query().Get("a") != "b" {
					t.Errorf("Backend Req: expected query a=b, got %q", req.URL.RawQuery)
				}
				if req.Header.Get("X-GFE") != "google.com" {
					t.Errorf("Backend Req: expected header X-GFE=google.com, got %q", req.Header.Get("X-GFE"))
				}
				body, _ := io.ReadAll(req.Body)
				if !bytes.Equal(body, []byte("thebody")) {
					t.Errorf("Backend Req: body mismatch, got %q", body)
				}
				return &http.Response{
					StatusCode: 201,
					Header:     http.Header{"Priority": []string{"High"}},
					Body:       io.NopCloser(bytes.NewReader([]byte("theresponsebody"))),
				}, nil
			},
		}

		config := DefaultClientConfig()
		config.ServerName = "foo"
		client := NewClient(config)

		relayClient := &http.Client{Transport: relayTransport}
		backendClient := &http.Client{Transport: backendTransport}

		err := client.localProxy(t.Context(), relayClient, backendClient)
		if err != nil {
			t.Errorf("Unexpected error: %v", err)
		}

		synctest.Wait()

		if relayRequests != 2 {
			t.Errorf("Expected 2 requests to relay, got %d", relayRequests)
		}
		if backendRequests != 1 {
			t.Errorf("Expected 1 request to backend, got %d", backendRequests)
		}
	})
}

func TestBackendError(t *testing.T) {
	synctest.Test(t, func(t *testing.T) {
		// We expect the response below to always contain 0 milliseconds.
		timeSince = func(t time.Time) time.Duration { return 0 * time.Millisecond }

		reqPayload, _ := proto.Marshal(&pb.HttpRequest{
			Id:     proto.String("15"),
			Method: proto.String("GET"),
			Url:    proto.String("http://invalid/foo/bar?a=b"),
			Header: []*pb.HttpHeader{{
				Name:  proto.String("X-GFE"),
				Value: proto.String("google.com")}},
			Body: []byte("thebody"),
		})

		respPayload, _ := proto.Marshal(&pb.HttpResponse{
			Id:                proto.String("15"),
			StatusCode:        proto.Int32(400),
			Body:              []byte("theresponsebody"),
			Eof:               proto.Bool(true),
			BackendDurationMs: proto.Int64(0),
		})

		relayRequests := 0
		relayTransport := &mockTransport{
			roundTrip: func(req *http.Request) (*http.Response, error) {
				relayRequests++
				switch relayRequests {
				case 1:
					if req.Method != "GET" || req.URL.Path != "/server/request" {
						t.Errorf("Relay Req 1: expected GET /server/request, got %s %s", req.Method, req.URL.Path)
					}
					return &http.Response{
						StatusCode: 200,
						Body:       io.NopCloser(bytes.NewReader(reqPayload)),
					}, nil
				case 2:
					if req.Method != "POST" || req.URL.Path != "/server/response" {
						t.Errorf("Relay Req 2: expected POST /server/response, got %s %s", req.Method, req.URL.Path)
					}
					body, _ := io.ReadAll(req.Body)
					if !bytes.Equal(body, respPayload) {
						t.Errorf("Relay Req 2: body mismatch")
					}
					return &http.Response{
						StatusCode: 200,
						Body:       io.NopCloser(strings.NewReader("")),
					}, nil
				default:
					t.Errorf("Unexpected extra request to relay: %s %s", req.Method, req.URL.Path)
					return nil, fmt.Errorf("unexpected request")
				}
			},
		}

		backendRequests := 0
		backendTransport := &mockTransport{
			roundTrip: func(req *http.Request) (*http.Response, error) {
				backendRequests++
				if req.Method != "GET" || req.URL.Path != "/foo/bar" {
					t.Errorf("Backend Req: expected GET /foo/bar, got %s %s", req.Method, req.URL.Path)
				}
				return &http.Response{
					StatusCode: 400,
					Body:       io.NopCloser(bytes.NewReader([]byte("theresponsebody"))),
				}, nil
			},
		}

		config := DefaultClientConfig()
		config.ServerName = "foo"
		client := NewClient(config)

		relayClient := &http.Client{Transport: relayTransport}
		backendClient := &http.Client{Transport: backendTransport}

		err := client.localProxy(t.Context(), relayClient, backendClient)
		if err != nil {
			t.Errorf("Unexpected error: %v", err)
		}

		synctest.Wait()

		if relayRequests != 2 {
			t.Errorf("Expected 2 requests to relay, got %d", relayRequests)
		}
		if backendRequests != 1 {
			t.Errorf("Expected 1 request to backend, got %d", backendRequests)
		}
	})
}

func TestServerTimeout(t *testing.T) {
	synctest.Test(t, func(t *testing.T) {
		relayRequests := 0
		relayTransport := &mockTransport{
			roundTrip: func(req *http.Request) (*http.Response, error) {
				relayRequests++
				if req.Method != "GET" || req.URL.Path != "/server/request" {
					t.Errorf("Relay Req: expected GET /server/request, got %s %s", req.Method, req.URL.Path)
				}
				return &http.Response{
					StatusCode: 408,
					Body:       io.NopCloser(strings.NewReader("")),
				}, nil
			},
		}

		config := DefaultClientConfig()
		config.ServerName = "foo"
		client := NewClient(config)

		relayClient := &http.Client{Transport: relayTransport}
		backendClient := &http.Client{} // not used

		err := client.localProxy(t.Context(), relayClient, backendClient)
		if err != ErrTimeout {
			t.Errorf("Unexpected error: %v", err)
		}

		synctest.Wait()

		if relayRequests != 1 {
			t.Errorf("Expected 1 request to relay, got %d", relayRequests)
		}
	})
}

func TestBuildResponsesTimesOut(t *testing.T) {
	bodyChannel := make(chan []byte)
	responseChannel := make(chan *pb.HttpResponse)
	resp := &pb.HttpResponse{
		Id:         proto.String("20"),
		StatusCode: proto.Int32(200),
	}
	config := DefaultClientConfig()
	config.BackendResponseTimeout = 10 * time.Millisecond
	client := NewClient(config)
	go client.buildResponses(bodyChannel, resp, responseChannel)
	bodyChannel <- []byte("foo")
	resp = <-responseChannel
	if got, want := resp.GetId(), "20"; got != want {
		t.Errorf("resp.Id = %q; want %q", got, want)
	}
	if got, want := resp.GetStatusCode(), int32(200); got != want {
		t.Errorf("resp.StatusCode = %d; want %d", got, want)
	}
	if got, want := string(resp.Body), "foo"; got != want {
		t.Errorf("resp.Body = %q; want %q", got, want)
	}
	if resp.Eof != nil {
		t.Errorf("resp.Eof = %v; want nil", *resp.Eof)
	}
	bodyChannel <- []byte("bar")
	resp = <-responseChannel
	if got, want := resp.GetId(), "20"; got != want {
		t.Errorf("resp.Id = %q; want %q", got, want)
	}
	if resp.StatusCode != nil {
		t.Errorf("resp.StatusCode = %d; want nil", *resp.StatusCode)
	}
	if got, want := string(resp.Body), "bar"; got != want {
		t.Errorf("resp.Body = %q; want %q", got, want)
	}
	if resp.Eof != nil {
		t.Errorf("resp.Eof = %v; want nil", *resp.Eof)
	}
	close(bodyChannel)
	resp = <-responseChannel
	if got, want := resp.GetId(), "20"; got != want {
		t.Errorf("resp.Id = %q; want %q", got, want)
	}
	if resp.StatusCode != nil {
		t.Errorf("resp.StatusCode = %d; want nil", *resp.StatusCode)
	}
	if got, want := string(resp.Body), ""; got != want {
		t.Errorf("resp.Body = %q; want %q", got, want)
	}
	if resp.Eof == nil || !*resp.Eof {
		t.Errorf("resp.Eof = %v; want true", resp.Eof)
	}
}
