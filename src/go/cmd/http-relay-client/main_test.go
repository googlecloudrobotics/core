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
	"net/http"
	"testing"
	"time"

	pb "github.com/googlecloudrobotics/core/src/proto/http-relay"

	. "github.com/onsi/gomega"
	"google.golang.org/protobuf/proto"
	"gopkg.in/h2non/gock.v1"
)

var defaultRelayURL = buildRelayURL()

func assertMocksDoneWithin(t *testing.T, d time.Duration) {
	for start := time.Now(); time.Since(start) < d; {
		if gock.IsDone() {
			return
		}
		time.Sleep(time.Millisecond)
	}
	for _, m := range gock.Pending() {
		t.Errorf("mock still pending after %s: %v", d, m.Request().URLStruct)
	}
}

func TestAssertMocksDoneWithin_SucceedsWhenMocksAreDone(t *testing.T) {
	assertMocksDoneWithin(t, time.Millisecond)
}

func TestAssertMocksDoneWithin_FailsWhenMocksNotDone(t *testing.T) {
	defer gock.Off()
	gock.New("https://localhost:8081")
	faket := &testing.T{}
	assertMocksDoneWithin(faket, time.Millisecond)
	if !faket.Failed() {
		t.Errorf("assertMocksDoneWithin didn't trigger an error despite outstanding mocks")
	}
}

func TestLocalProxy(t *testing.T) {
	// Hot patch: gock refuses to match bodies with unknown content-types by default.
	gock.BodyTypes = append(gock.BodyTypes, "application/vnd.google.protobuf;proto=cloudrobotics.http_relay.v1alpha1.HttpResponse")
	defer gock.Off()

	// We expect the response below to always contain 0 milliseconds.
	timeSince = func(t time.Time) time.Duration { return 0 * time.Millisecond }

	req, _ := proto.Marshal(&pb.HttpRequest{
		Id:     proto.String("15"),
		Method: proto.String("GET"),
		Url:    proto.String("http://invalid/foo/bar?a=b"),
		Header: []*pb.HttpHeader{{
			Name:  proto.String("X-GFE"),
			Value: proto.String("google.com")}},
		Body: []byte("thebody"),
	})
	resp, _ := proto.Marshal(&pb.HttpResponse{
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
	gock.New("https://localhost:8081").
		Get("/server/request").
		MatchParam("server", "foo").
		Reply(200).
		BodyString(string(req))
	gock.New("https://localhost:8080").
		Get("/foo/bar").
		MatchParam("a", "b").
		MatchHeader("X-GFE", "google.com").
		BodyString("thebody").
		Reply(201).
		SetHeader("Priority", "High").
		BodyString("theresponsebody")
	gock.New("https://localhost:8081").
		Post("/server/response").
		Body(bytes.NewReader(resp)).
		Reply(200)

	err := localProxy(&http.Client{}, &http.Client{}, defaultRelayURL)
	if err != nil {
		t.Errorf("Unexpected error: %s", err)
	}
	assertMocksDoneWithin(t, 10*time.Second)
}

func TestBackendError(t *testing.T) {
	// Hot patch: gock refuses to match bodies with unknown content-types by default.
	gock.BodyTypes = append(gock.BodyTypes, "application/vnd.google.protobuf;proto=cloudrobotics.http_relay.v1alpha1.HttpResponse")
	defer gock.Off()

	// We expect the response below to always contain 0 milliseconds.
	timeSince = func(t time.Time) time.Duration { return 0 * time.Millisecond }

	// The pending request on the relay-server side.
	req, _ := proto.Marshal(&pb.HttpRequest{
		Id:     proto.String("15"),
		Method: proto.String("GET"),
		Url:    proto.String("http://invalid/foo/bar?a=b"),
		Header: []*pb.HttpHeader{{
			Name:  proto.String("X-GFE"),
			Value: proto.String("google.com")}},
		Body: []byte("thebody"),
	})

	resp, _ := proto.Marshal(&pb.HttpResponse{
		Id:                proto.String("15"),
		StatusCode:        proto.Int32(400),
		Body:              []byte("theresponsebody"),
		Eof:               proto.Bool(true),
		BackendDurationMs: proto.Int64(0),
	})

	relayServerAddress := "https://localhost:8081"
	backendServerAddress := "https://localhost:8080"

	// Mocks the response from the relay server from which we are getting
	// the initial data.
	gock.New(relayServerAddress).
		Get("/server/request").
		MatchParam("server", "foo").
		Reply(200).
		BodyString(string(req))

	// Mocks the response from the backend server to which we relayed data.
	gock.New(backendServerAddress).
		Get("/foo/bar").
		MatchParam("a", "b").
		MatchHeader("X-GFE", "google.com").
		BodyString("thebody").
		Reply(400).
		BodyString("theresponsebody")

	// Mocks the response from the realy-server after having received the
	// actual backend response.
	gock.New(relayServerAddress).
		Post("/server/response").
		Body(bytes.NewReader(resp)).
		Reply(200)

	// localProxy ...
	// 1. pulls a request from the realy-server (/server/request)
	// 2. send that request to the backend server (here localhost:8080/foo/bar?a=b)
	// 3. retrieves the response from the backend and sends it to the relay-server
	err := localProxy(&http.Client{}, &http.Client{}, defaultRelayURL)
	if err != nil {
		t.Errorf("Unexpected error: %s", err)
	}
	assertMocksDoneWithin(t, 10*time.Second)
}

func TestServerTimeout(t *testing.T) {
	// Hot patch: gock refuses to match bodies with application/octet-data
	// by default.
	gock.BodyTypes = append(gock.BodyTypes, "application/octet-data")
	defer gock.Off()

	req, _ := proto.Marshal(&pb.HttpRequest{
		Id:     proto.String("15"),
		Method: proto.String("GET"),
		Url:    proto.String("http://invalid/foo/bar?a=b"),
		Header: []*pb.HttpHeader{{
			Name:  proto.String("X-GFE"),
			Value: proto.String("google.com")}},
		Body: []byte("thebody"),
	})
	gock.New("https://localhost:8081").
		Get("/server/request").
		MatchParam("server", "foo").
		Reply(408).
		BodyString(string(req))

	err := localProxy(&http.Client{}, &http.Client{}, defaultRelayURL)
	if err != ErrTimeout {
		t.Errorf("Unexpected error: %s", err)
	}
	assertMocksDoneWithin(t, 10*time.Second)
}

func TestBuildResponsesTimesOut(t *testing.T) {
	g := NewGomegaWithT(t)
	bodyChannel := make(chan []byte)
	responseChannel := make(chan *pb.HttpResponse)
	resp := &pb.HttpResponse{
		Id:         proto.String("20"),
		StatusCode: proto.Int32(200),
	}
	go buildResponses(bodyChannel, resp, responseChannel, 10*time.Millisecond)
	bodyChannel <- []byte("foo")
	resp = <-responseChannel
	g.Expect(*resp.Id).To(Equal("20"))
	g.Expect(*resp.StatusCode).To(Equal(int32(200)))
	g.Expect(string(resp.Body)).To(Equal("foo"))
	g.Expect(resp.Eof).To(BeNil())
	bodyChannel <- []byte("bar")
	resp = <-responseChannel
	g.Expect(*resp.Id).To(Equal("20"))
	g.Expect(resp.StatusCode).To(BeNil())
	g.Expect(string(resp.Body)).To(Equal("bar"))
	g.Expect(resp.Eof).To(BeNil())
	close(bodyChannel)
	resp = <-responseChannel
	g.Expect(*resp.Id).To(Equal("20"))
	g.Expect(resp.StatusCode).To(BeNil())
	g.Expect(string(resp.Body)).To(Equal(""))
	g.Expect(*resp.Eof).To(Equal(true))
}
