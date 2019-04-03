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

	pb "src/proto/http-relay"

	"github.com/golang/protobuf/proto"
	. "github.com/onsi/gomega"
	"gopkg.in/h2non/gock.v1"
)

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
	resp, _ := proto.Marshal(&pb.HttpResponse{
		Id:         proto.String("15"),
		StatusCode: proto.Int(201),
		Header: []*pb.HttpHeader{
			{
				Name:  proto.String("Priority"),
				Value: proto.String("High"),
			},
		},
		Body: []byte("theresponsebody"),
		Eof:  proto.Bool(true),
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

	err := localProxy(&http.Client{}, &http.Client{})
	if err != nil {
		t.Errorf("Unexpected error: %s", err)
	}
	assertMocksDoneWithin(t, 10*time.Second)
}

func TestBackendError(t *testing.T) {
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
	resp, _ := proto.Marshal(&pb.HttpResponse{
		Id:         proto.String("15"),
		StatusCode: proto.Int(400),
		Body:       []byte("theresponsebody"),
		Eof:        proto.Bool(true),
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
		Reply(400).
		BodyString("theresponsebody")
	gock.New("https://localhost:8081").
		Post("/server/response").
		Body(bytes.NewReader(resp)).
		Reply(200)

	err := localProxy(&http.Client{}, &http.Client{})
	if err != nil {
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
