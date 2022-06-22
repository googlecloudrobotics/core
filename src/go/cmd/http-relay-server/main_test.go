// Copyright 2019 The Cloud Robotics Authors
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
	"io/ioutil"
	"net/http"
	"net/http/httptest"
	"strings"
	"sync"
	"testing"

	pb "github.com/googlecloudrobotics/core/src/proto/http-relay"

	hijacktest "github.com/getlantern/httptest"
	"github.com/golang/protobuf/proto"
)

func checkResponse(t *testing.T, resp *http.Response, wantStatus int, wantBody string) {
	t.Helper()
	if want, got := wantStatus, resp.StatusCode; want != got {
		t.Errorf("Wrong response code; want %d; got %d", want, got)
	}
	body, err := ioutil.ReadAll(resp.Body)
	if err != nil {
		t.Errorf("Failed to read body stream")
	}
	if want, got := wantBody, string(body); want != got {
		t.Errorf("Wrong body; want %s; got %s", want, got)
	}
}

func TestClientHandler(t *testing.T) {
	req := httptest.NewRequest("GET", "/client/foo/bar?a=b#c", strings.NewReader("body"))
	req.Header.Add("X-Deadline", "now")
	respRecorder := httptest.NewRecorder()
	server := newServer()
	wg := sync.WaitGroup{}
	wg.Add(1)
	go func() { server.client(respRecorder, req); wg.Done() }()
	relayRequest, err := server.b.GetRequest(context.Background(), "foo")
	if err != nil {
		t.Errorf("Error when getting request: %v", err)
	}

	wantRequest := &pb.HttpRequest{
		Id:     relayRequest.Id,
		Method: proto.String("GET"),
		Host:   proto.String("example.com"),
		Url:    proto.String("http://invalid/bar?a=b#c"),
		Header: []*pb.HttpHeader{{
			Name:  proto.String("X-Deadline"),
			Value: proto.String("now"),
		}},
		Body: []byte("body"),
	}
	if !proto.Equal(wantRequest, relayRequest) {
		t.Errorf("Wrong encapsulated request; want %s; got '%s'", wantRequest, relayRequest)
	}

	server.b.SendResponse(&pb.HttpResponse{
		Id:         relayRequest.Id,
		StatusCode: proto.Int(201),
		Header: []*pb.HttpHeader{{
			Name:  proto.String("X-GFE"),
			Value: proto.String("google.com"),
		}},
		Body: []byte("the"),
	})

	server.b.SendResponse(&pb.HttpResponse{
		Id:   relayRequest.Id,
		Body: []byte("body"),
		Eof:  proto.Bool(true),
	})

	wg.Wait()
	resp := respRecorder.Result()
	checkResponse(t, resp, 201, "thebody")
	if want, got := 1, len(resp.Header); want != got {
		t.Errorf("Wrong # of headers; want %d; got %d", want, got)
	}
	if want, got := "google.com", resp.Header.Get("X-GFE"); want != got {
		t.Errorf("Wrong header value; want %s; got %s", want, got)
	}
}

func TestClientBadRequest(t *testing.T) {
	tests := []struct {
		desc     string
		req      *http.Request
		wantCode int
		wantMsg  string
	}{
		{
			desc:     "url-path misses the backend name and path",
			req:      httptest.NewRequest("GET", "/client/", strings.NewReader("body")),
			wantCode: 400,
			wantMsg:  "Request path too short:",
		},
		{
			desc:     "url-path misses the backend header",
			req:      httptest.NewRequest("GET", "/", strings.NewReader("body")),
			wantCode: 400,
			wantMsg:  "Request without required header:",
		},
	}

	for _, tc := range tests {
		t.Run(tc.desc, func(t *testing.T) {
			respRecorder := httptest.NewRecorder()
			server := newServer()
			wg := sync.WaitGroup{}
			wg.Add(1)
			go func() { server.client(respRecorder, tc.req); wg.Done() }()
			wg.Wait()

			resp := respRecorder.Result()
			if got := resp.StatusCode; tc.wantCode != got {
				t.Errorf("Wrong response code; want %d; got %d", tc.wantCode, got)
			}
			if tc.wantMsg != "" {
				body, err := ioutil.ReadAll(resp.Body)
				if err != nil {
					t.Errorf("Failed to read body stream: %s", err)
				}
				if !strings.Contains(string(body), tc.wantMsg) {
					t.Errorf("Wrong response body; want %q; got %q", tc.wantMsg, body)
				}
			}
		})
	}
}

func nonRepeatingByteArray(n int) []byte {
	result := make([]byte, 0, n)
	i := 0
	for len(result) < n {
		result = append(result, []byte(fmt.Sprintf("%8d", i))...)
		i += 1
	}
	return result
}

func TestRequestStreamHandler(t *testing.T) {
	// Use a large request stream body to ensure it gets split into multiple
	// blocks. This would have caught a race that jumbles the request stream.
	oldBlockSize := *blockSize
	*blockSize = 64
	defer func() {
		*blockSize = oldBlockSize
	}()
	wantRequestStream := nonRepeatingByteArray(3 * (*blockSize))

	// In a background goroutine, run a client request with post-request data
	// in the request stream.
	req := httptest.NewRequest("GET", "/client/foo/bar?a=b#c", strings.NewReader("body"))
	req.Header.Add("X-Deadline", "now")
	respRecorder := hijacktest.NewRecorder(wantRequestStream)
	server := newServer()
	wg := sync.WaitGroup{}
	wg.Add(1)
	go func() { server.client(respRecorder, req); wg.Done() }()

	// Simulate a 101 Switching Protocols response from the backend.
	relayRequest, err := server.b.GetRequest(context.Background(), "foo")
	if err != nil {
		t.Errorf("Error when getting request: %v", err)
	}
	server.b.SendResponse(&pb.HttpResponse{
		Id:         relayRequest.Id,
		StatusCode: proto.Int(101),
		Header: []*pb.HttpHeader{{
			Name:  proto.String("Upgrade"),
			Value: proto.String("SPDY/3.1"),
		}},
		Body: []byte("the"),
	})

	// Get the data from the request stream and check its contents.
	gotRequestStream := []byte{}
	for len(gotRequestStream) < len(wantRequestStream) {
		reqstreamRecorder := httptest.NewRecorder()
		streamreq := httptest.NewRequest("POST", "/server/requeststream?id="+*relayRequest.Id, nil)
		server.serverRequestStream(reqstreamRecorder, streamreq)
		switch sc := reqstreamRecorder.Result().StatusCode; sc {
		case http.StatusOK:
			gotRequestStream = append(gotRequestStream, reqstreamRecorder.Body.Bytes()...)
		case http.StatusGone:
			break
		default:
			t.Errorf("POST /server/requeststream returned unexpected status %d, want %d or %d", sc, http.StatusOK, http.StatusGone)
		}
	}
	if !bytes.Equal(wantRequestStream, gotRequestStream) {
		t.Errorf("POST /server/requeststream returned unexpected data, got:\n%s\nwant:\n%s", gotRequestStream, wantRequestStream)
	}

	// Terminate the client request and verify the response.
	server.b.SendResponse(&pb.HttpResponse{
		Id:   relayRequest.Id,
		Body: []byte("body"),
		Eof:  proto.Bool(true),
	})
	wg.Wait()
	checkResponse(t, respRecorder.Result(), 101, "thebody")
}

func TestServerRequestResponseHandler(t *testing.T) {
	backendReq := &pb.HttpRequest{
		Id:     proto.String("15"),
		Method: proto.String("GET"),
		Url:    proto.String("http://invalid/my/url"),
		Header: []*pb.HttpHeader{{
			Name:  proto.String("X-GFE"),
			Value: proto.String("google.com"),
		}},
		Body: []byte("thebody"),
	}

	backendResp := &pb.HttpResponse{
		Id:         backendReq.Id,
		StatusCode: proto.Int(201),
		Header: []*pb.HttpHeader{{
			Name:  proto.String("X-GFE"),
			Value: proto.String("google.com"),
		}},
		Body: []byte("thebody"),
		Eof:  proto.Bool(true),
	}
	backendRespBody, err := proto.Marshal(backendResp)
	if err != nil {
		t.Errorf("Failed to marshal test response: %s", err)
	}

	req := httptest.NewRequest("GET", "/server/request?server=b", strings.NewReader(""))
	resp := httptest.NewRequest("POST", "/server/response", bytes.NewReader(backendRespBody))
	reqRecorder := httptest.NewRecorder()
	respRecorder := httptest.NewRecorder()
	server := newServer()
	wg := sync.WaitGroup{}
	wg.Add(1)
	go func() {
		server.serverRequest(reqRecorder, req)
		server.serverResponse(respRecorder, resp)
		wg.Done()
	}()

	serverRespChan, err := server.b.RelayRequest("b", backendReq)
	if err != nil {
		t.Errorf("Got relay request error: %v", err)
	}
	serverResp := <-serverRespChan
	wg.Wait()

	if want, got := 200, reqRecorder.Result().StatusCode; want != got {
		t.Errorf("Wrong response code; want %d; got %d", want, got)
	}
	body, err := ioutil.ReadAll(reqRecorder.Result().Body)
	if err != nil {
		t.Errorf("Failed to read body stream: %s", err)
	}
	if !strings.Contains(string(body), "/my/url") {
		t.Errorf("Serialize request didn't contain URL: %s", string(body))
	}
	if !strings.Contains(string(body), "X-GFE") {
		t.Errorf("Serialize request didn't contain header: %s", string(body))
	}

	if want, got := 200, respRecorder.Result().StatusCode; want != got {
		t.Errorf("Wrong response code; want %d; got %d", want, got)
	}

	if !proto.Equal(serverResp, backendResp) {
		t.Errorf("Encapsulated response was garbled; want %s; got %s", backendResp, serverResp)
	}
}

func TestServerResponseHandlerWithInvalidRequestID(t *testing.T) {
	backendResp := &pb.HttpResponse{
		Id:         proto.String("not found"),
		StatusCode: proto.Int(201),
		Header: []*pb.HttpHeader{{
			Name:  proto.String("X-GFE"),
			Value: proto.String("google.com"),
		}},
		Body: []byte("thebody"),
		Eof:  proto.Bool(true),
	}
	backendRespBody, err := proto.Marshal(backendResp)
	if err != nil {
		t.Errorf("Failed to marshal test response: %s", err)
	}

	resp := httptest.NewRequest("POST", "/server/response", bytes.NewReader(backendRespBody))
	respRecorder := httptest.NewRecorder()
	server := newServer()
	server.serverResponse(respRecorder, resp)

	if want, got := http.StatusBadRequest, respRecorder.Result().StatusCode; want != got {
		t.Errorf("serverResponse() gave wrong status code; want %d; got %d", want, got)
	}
}
