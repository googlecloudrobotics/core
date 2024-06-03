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

package server

import (
	"bytes"
	"context"
	"log/slog"
	"sync"
	"testing"
	"time"

	pb "github.com/googlecloudrobotics/core/src/proto/http-relay"
	"google.golang.org/protobuf/proto"
)

const (
	// These IDs are used for testing multiple requests. The contents of the
	// strings are not important, as long as they are unique.
	idOne     = "idOne"
	idTwo     = "idTwo"
	idThree   = "idThree"
	unknownID = "unknownID"
)

// brokerConn helps manage send/receive operations on the broker. Because the broker
// will immediately reject user clients for a backend which it has not seen before,
// we need to ensure that the backend side makes the connection first.
//
// Only one request id can be used for each `brokerConn`
type brokerConn struct {
	ready chan struct{}
}

func newBrokerConn() brokerConn {
	return brokerConn{
		ready: make(chan struct{}),
	}
}

func (bt *brokerConn) runSender(t *testing.T, b *broker, s string, m string, wg *sync.WaitGroup) {
	<-bt.ready
	respChan, err := b.RelayRequest(s, &pb.HttpRequest{Id: proto.String(m), Url: proto.String("http://example.com/foo")})
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

func (bt *brokerConn) bakeRequest(b *broker, s string) {
	// ensure that the request is created before the user client connects
	b.m.Lock()
	if _, found := b.req[s]; !found {
		b.req[s] = make(chan *pb.HttpRequest)
	}
	b.m.Unlock()
	close(bt.ready)
}

func (bt *brokerConn) runReceiver(t *testing.T, b *broker, s string, wg *sync.WaitGroup) {
	bt.bakeRequest(b, s)
	req, err := b.GetRequest(context.Background(), s, "/")
	if err != nil {
		t.Errorf("Error when getting request: %v", err)
	}
	err = b.SendResponse(&pb.HttpResponse{Id: req.Id, Body: []byte(*req.Id), Eof: proto.Bool(true)})
	if err != nil {
		t.Errorf("Error when sending response: %v", err)
	}
	wg.Done()
}

// runSenderStream expects two items in the response stream, and it doesn't care about the contents.
func (bt *brokerConn) runSenderStream(t *testing.T, b *broker, s string, m string, wg *sync.WaitGroup) {
	<-bt.ready
	respChan, err := b.RelayRequest(s, &pb.HttpRequest{Id: proto.String(m), Url: proto.String("http://example.com/foo")})
	if err != nil {
		t.Errorf("Got relay request error: %v", err)
	}
	// First response (Eof=false)
	if _, more := <-respChan; !more {
		t.Errorf("Got zero responses, want two.")
	}
	// Second response (Eof=true)
	if _, more := <-respChan; !more {
		t.Errorf("Got one response, want two.")
	}
	// Check that channel is closed.
	if _, more := <-respChan; more {
		t.Errorf("Got more than two responses, want two.")
	}
	wg.Done()
}

// runReceiverStream sends two items in the response stream, waiting before the second.
// It returns after the first response has been sent.
func (bt *brokerConn) runReceiverStream(t *testing.T, b *broker, s string, wg *sync.WaitGroup, done <-chan bool) {
	bt.bakeRequest(b, s)
	req, err := b.GetRequest(context.Background(), s, "/")
	if err != nil {
		t.Errorf("Error when getting request: %v", err)
	}
	err = b.SendResponse(&pb.HttpResponse{Id: req.Id, Body: []byte(*req.Id), Eof: proto.Bool(false)})
	if err != nil {
		t.Errorf("Error when sending response: %v", err)
	}
	go func() {
		<-done
		err = b.SendResponse(&pb.HttpResponse{Id: req.Id, Body: []byte(*req.Id), Eof: proto.Bool(true)})
		if err != nil {
			t.Errorf("Error when sending response: %v", err)
		}
		wg.Done()
	}()
}

func TestNormalCase(t *testing.T) {
	b := newBroker()
	var bConns [3]brokerConn
	for i := range bConns {
		bConns[i] = newBrokerConn()
	}

	var wg sync.WaitGroup
	wg.Add(6)
	go bConns[0].runSender(t, b, "foo", idOne, &wg)
	go bConns[1].runSender(t, b, "foo", idTwo, &wg)
	go bConns[2].runSender(t, b, "bar", idThree, &wg)
	go bConns[0].runReceiver(t, b, "foo", &wg)
	go bConns[1].runReceiver(t, b, "foo", &wg)
	go bConns[2].runReceiver(t, b, "bar", &wg)
	wg.Wait()
}

func TestResponseStream(t *testing.T) {
	b := newBroker()
	bc := newBrokerConn()
	var wg sync.WaitGroup
	done := make(chan bool)
	wg.Add(2)
	go bc.runSenderStream(t, b, "foo", idOne, &wg)
	bc.runReceiverStream(t, b, "foo", &wg, done)
	done <- true
	wg.Wait()
}

func TestMissingId(t *testing.T) {
	b := newBroker()
	err := b.SendResponse(&pb.HttpResponse{Id: proto.String(idOne)})
	if err == nil {
		t.Errorf("Invalid response did not produce an error")
	}
}

func TestDuplicateId(t *testing.T) {
	b := newBroker()
	bc := newBrokerConn()
	var wg sync.WaitGroup
	wg.Add(2)
	go bc.runSender(t, b, "foo", idOne, &wg)
	go bc.runReceiver(t, b, "foo", &wg)
	wg.Wait()

	err := b.SendResponse(&pb.HttpResponse{Id: proto.String(idOne)})
	if err == nil {
		t.Errorf("Duplicate response did not produce an error")
	}
}

func TestRequestStream(t *testing.T) {
	// Start a request that won't terminate until we send `done`.
	b := newBroker()
	bc := newBrokerConn()
	var wg sync.WaitGroup
	done := make(chan bool)
	wg.Add(2)
	go bc.runSenderStream(t, b, "foo", idOne, &wg)
	bc.runReceiverStream(t, b, "foo", &wg, done)

	// Send a message over the request stream and assert that it arrives.
	go func() {
		ok := b.PutRequestStream(idOne, []byte("hello"))
		if !ok {
			t.Error("PutRequestStream(idOne, \"hello\") = false, want true")
		}
	}()
	data, ok := b.GetRequestStream(idOne)
	if !ok {
		t.Error("data, ok := GetRequestStream(idOne); ok = false, want true")
	}
	if !bytes.Equal(data, []byte("hello")) {
		t.Errorf("data, ok := GetRequestStream(idOne); data = %q, want \"hello\"", data)
	}

	// Complete the ongoing request.
	done <- true
	wg.Wait()
}

func TestRequestStreamUnknownID(t *testing.T) {
	b := newBroker()
	if ok := b.PutRequestStream(unknownID, []byte{}); ok {
		t.Error("ok := PutRequestStream(unknownID, \"\"); ok = true, want false")
	}
	if _, ok := b.GetRequestStream(unknownID); ok {
		t.Error("_, ok := GetRequestStream(unknownID; ok = true, want false")
	}
}

func TestTimeout(t *testing.T) {
	b := newBroker()
	// create the request channel manually to avoid race condition between the 2
	// goroutines below
	b.req["foo"] = make(chan *pb.HttpRequest)

	var wg sync.WaitGroup
	wg.Add(2)
	go func() {
		respChan, err := b.RelayRequest("foo", &pb.HttpRequest{Id: proto.String(idOne), Url: proto.String("http://example.com/foo")})
		if err != nil {
			t.Errorf("Got relay request error: %v", err)
		}
		if _, more := <-respChan; more {
			t.Errorf("Got unexpected response")
		}
		wg.Done()
	}()
	go func() {
		slog.Info("Getting request")
		b.GetRequest(context.Background(), "foo", "/")
		slog.Info("Reaping inactive requests")
		b.ReapInactiveRequests(time.Now().Add(10 * time.Second))
		slog.Info("Done")
		wg.Done()
	}()
	wg.Wait()
}
