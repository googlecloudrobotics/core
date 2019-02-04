// Copyright 2019 The Google Cloud Robotics Authors
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
	"log"
	"sync"
	"testing"
	"time"

	pb "src/proto/http-relay"

	"github.com/golang/protobuf/proto"
)

func runSender(t *testing.T, b *broker, s string, m string, wg *sync.WaitGroup) {
	respChan, err := b.RelayRequest(s, &pb.HttpRequest{Id: proto.String(m)})
	if err != nil {
		t.Errorf("Got relay request error: %v", err)
	}
	resp, more := <-respChan
	if want, got := m, string(resp.Body); want != got {
		t.Errorf("Wrong response; want %s; got %s\n", want, got)
	}
	resp, more = <-respChan
	if more {
		t.Errorf("Got more than 1 response: %v", resp)
	}
	wg.Done()
}

func runReceiver(t *testing.T, b *broker, s string, wg *sync.WaitGroup) {
	req, err := b.GetRequest(s)
	if err != nil {
		t.Errorf("Error when getting request: %s", err)
	}
	err = b.SendResponse(&pb.HttpResponse{Id: req.Id, Body: []byte(*req.Id), Eof: proto.Bool(true)})
	if err != nil {
		t.Errorf("Error when sending response: %s", err)
	}
	wg.Done()
}

func TestNormalCase(t *testing.T) {
	b := newBroker()
	var wg sync.WaitGroup
	wg.Add(6)
	go runSender(t, b, "foo", "15", &wg)
	go runSender(t, b, "foo", "16", &wg)
	go runSender(t, b, "bar", "17", &wg)
	go runReceiver(t, b, "foo", &wg)
	go runReceiver(t, b, "foo", &wg)
	go runReceiver(t, b, "bar", &wg)
	wg.Wait()
}

func TestMissingId(t *testing.T) {
	b := newBroker()
	err := b.SendResponse(&pb.HttpResponse{Id: proto.String("15")})
	if err == nil {
		t.Errorf("Invalid response did not produce an error")
	}
}

func TestDuplicateId(t *testing.T) {
	b := newBroker()
	var wg sync.WaitGroup
	wg.Add(2)
	go runSender(t, b, "foo", "15", &wg)
	go runReceiver(t, b, "foo", &wg)
	wg.Wait()

	err := b.SendResponse(&pb.HttpResponse{Id: proto.String("15")})
	if err == nil {
		t.Errorf("Duplicate response did not produce an error")
	}
}

func TestTimeout(t *testing.T) {
	b := newBroker()
	var wg sync.WaitGroup
	wg.Add(2)
	go func() {
		respChan, err := b.RelayRequest("foo", &pb.HttpRequest{Id: proto.String("15")})
		if err != nil {
			t.Errorf("Got relay request error: %v", err)
		}
		if _, more := <-respChan; more {
			t.Errorf("Got unexpected response")
		}
		wg.Done()
	}()
	go func() {
		log.Printf("Getting request")
		b.GetRequest("foo")
		log.Printf("Reaping inactive requests")
		b.ReapInactiveRequests(time.Now().Add(10 * time.Second))
		log.Printf("Done")
		wg.Done()
	}()
	wg.Wait()
}
