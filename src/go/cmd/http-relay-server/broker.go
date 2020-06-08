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
	"fmt"
	"log"
	pb "src/proto/http-relay"
	"sync"
	"time"

	"github.com/prometheus/client_golang/prometheus"
)

var (
	brokerRequests = prometheus.NewCounterVec(
		prometheus.CounterOpts{
			Name: "broker_requests",
			Help: "Number of requests to the broker",
		},
		[]string{"method"},
	)
	brokerResponses = prometheus.NewCounterVec(
		prometheus.CounterOpts{
			Name: "broker_responses",
			Help: "Number of responses from the broker",
		},
		[]string{"method", "result"},
	)
)

func init() {
	prometheus.MustRegister(brokerRequests)
	prometheus.MustRegister(brokerResponses)
}

type pendingRequest struct {
	requestStream  chan []byte
	responseStream chan *pb.HttpResponse
	lastActivity   time.Time
}

// broker implements a thread-safe map for the request and response queues.
type broker struct {
	m    sync.Mutex
	req  map[string]chan *pb.HttpRequest
	resp map[string]*pendingRequest
}

func newBroker() *broker {
	var r broker
	r.req = make(map[string]chan *pb.HttpRequest)
	r.resp = make(map[string]*pendingRequest)
	return &r
}

// RelayRequest matches a pending relay client's request to the encapsulated
// request and returns a channel for the results.
func (r *broker) RelayRequest(server string, request *pb.HttpRequest) (<-chan *pb.HttpResponse, error) {
	id := *request.Id

	r.m.Lock()
	if r.req[server] == nil {
		r.req[server] = make(chan *pb.HttpRequest)
	}
	if r.resp[id] != nil {
		return nil, fmt.Errorf("Multiple clients trying to handle request ID %s", id)
	}
	r.resp[id] = &pendingRequest{
		requestStream:  make(chan []byte),
		responseStream: make(chan *pb.HttpResponse),
		lastActivity:   time.Now(),
	}
	reqChan := r.req[server]
	respChan := r.resp[id].responseStream
	r.m.Unlock()

	log.Printf("Enqueuing request %s for %s", id, server)
	brokerRequests.WithLabelValues("client").Inc()
	select {
	case reqChan <- request:
		return respChan, nil
	case <-time.After(10 * time.Second):
		return nil, fmt.Errorf("Timeout waiting for relay client to accept request for %s", server)
	}
}

// GetRequest obtains a client's request for the server identifier. It blocks
// until a client makes a request.
func (r *broker) GetRequest(server string) (*pb.HttpRequest, error) {
	r.m.Lock()
	if r.req[server] == nil {
		r.req[server] = make(chan *pb.HttpRequest)
	}
	reqChan := r.req[server]
	r.m.Unlock()

	brokerRequests.WithLabelValues("server_request").Inc()
	select {
	case req := <-reqChan:
		brokerResponses.WithLabelValues("server_request", "ok").Inc()
		return req, nil
	case <-time.After(time.Second * 30):
		brokerResponses.WithLabelValues("server_request", "timeout").Inc()
		return nil, fmt.Errorf("No request received within timeout")
	}
}

// GetRequestStream gets data from the stream that follows a client's HTTP
// request. For example, when using `kubectl exec` this passes stdin data from
// the broker to the relay client.
// If no ongoing request matches the given ID, this returns ok=false.
func (r *broker) GetRequestStream(id string) ([]byte, bool) {
	r.m.Lock()
	pr := r.resp[id]
	r.m.Unlock()
	if pr == nil {
		return nil, false
	}

	select {
	case req := <-pr.requestStream:
		return req, true
	case <-time.After(time.Second * 30):
		return []byte{}, true
	}
}

// PutsRequestStream adds data from the stream that follows a client's HTTP
// request. For example, when using `kubectl exec` this passes stdin data from
// kubectl to the broker.
// If no ongoing request matches the given ID, this returns ok=false.
func (r *broker) PutRequestStream(id string, data []byte) bool {
	r.m.Lock()
	pr := r.resp[id]
	r.m.Unlock()
	if pr == nil {
		return false
	}

	pr.requestStream <- data
	return true
}

// SendResponse delivers the HttpResponse to the client handler that created the
// request. It fails if and only if the request ID is not recognized.
func (r *broker) SendResponse(resp *pb.HttpResponse) error {
	id := *resp.Id
	r.m.Lock()
	pr := r.resp[id]
	if pr == nil {
		r.m.Unlock()
		brokerResponses.WithLabelValues("server_response", "invalid").Inc()
		return fmt.Errorf("Duplicate or invalid request ID %s", id)
	}
	if resp.GetEof() {
		delete(r.resp, id)
	} else {
		pr.lastActivity = time.Now()
	}
	r.m.Unlock()
	brokerRequests.WithLabelValues("server_response").Inc()
	log.Printf("Delivering response %s to client", id)
	pr.responseStream <- resp
	if resp.GetEof() {
		close(pr.responseStream)
	}
	brokerResponses.WithLabelValues("server_response", "ok").Inc()
	return nil
}

func (r *broker) ReapInactiveRequests(threshold time.Time) {
	r.m.Lock()
	for key, value := range r.resp {
		if value.lastActivity.Before(threshold) {
			log.Printf("Timeout on inactive request %s", key)
			defer close(value.requestStream)
			defer close(value.responseStream)
			// Amazingly, this is safe in Go: https://stackoverflow.com/questions/23229975/is-it-safe-to-remove-selected-keys-from-map-within-a-range-loop
			delete(r.resp, key)
		}
	}
	r.m.Unlock()
}
