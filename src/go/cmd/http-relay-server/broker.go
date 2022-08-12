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
	"context"
	"fmt"
	"log"
	"strings"
	"sync"
	"time"

	pb "github.com/googlecloudrobotics/core/src/proto/http-relay"

	"github.com/prometheus/client_golang/prometheus"
)

var (
	brokerRequests = prometheus.NewCounterVec(
		prometheus.CounterOpts{
			Name: "broker_requests",
			Help: "Number of requests to the broker",
		},
		[]string{"method", "backend"},
	)
	brokerResponses = prometheus.NewCounterVec(
		prometheus.CounterOpts{
			Name: "broker_responses",
			Help: "Number of responses from the broker",
		},
		[]string{"method", "result", "backend"},
	)
	brokerResponseDurations = prometheus.NewHistogramVec(
		prometheus.HistogramOpts{
			Name: "broker_responses_durations",
			Help: "Time from request to final response in ms",
		},
		[]string{"method", "backend"},
	)
)

func init() {
	prometheus.MustRegister(brokerRequests)
	prometheus.MustRegister(brokerResponses)
	prometheus.MustRegister(brokerResponseDurations)
}

type pendingResponse struct {
	// for comunication of relay-server to relay-cleint
	requestStream chan []byte
	// for communication of relay-server to client
	responseStream chan *pb.HttpResponse
	lastActivity   time.Time
	// For diagnostics only.
	startTime time.Time
}

// broker implements a thread-safe map for the request and response queues.
// Requests (req) are mapped by server-name. There is only channel per relay-
// client (identified by the server query parameter)
// Responses (resp) are mapped by stream id (randomly generated hex string).
// There can be multiple concurrent transfers per relay-client, each identified
// by a unique id query parameter.
type broker struct {
	m    sync.Mutex
	req  map[string]chan *pb.HttpRequest
	resp map[string]*pendingResponse
}

func newBroker() *broker {
	var r broker
	r.req = make(map[string]chan *pb.HttpRequest)
	r.resp = make(map[string]*pendingResponse)
	return &r
}

// Healthy can be used for server health checks. If the server is deadlocked it
// will block forever.
func (r *broker) Healthy() error {
	r.m.Lock()
	defer r.m.Unlock()
	return nil
}

// RelayRequest matches a pending relay client's request to the encapsulated
// request and returns a channel for the results.
func (r *broker) RelayRequest(server string, request *pb.HttpRequest) (<-chan *pb.HttpResponse, error) {
	id := *request.Id

	r.m.Lock()
	if r.req[server] == nil {
		// This happens when the relay-client connects for the first time.
		r.req[server] = make(chan *pb.HttpRequest)
	}
	if r.resp[id] != nil {
		return nil, fmt.Errorf("Multiple clients trying to handle request ID %s on server %s", id, server)
	}
	ts := time.Now()
	r.resp[id] = &pendingResponse{
		requestStream:  make(chan []byte),
		responseStream: make(chan *pb.HttpResponse),
		lastActivity:   ts,
		startTime:      ts,
	}
	reqChan := r.req[server]
	respChan := r.resp[id].responseStream
	r.m.Unlock()

	log.Printf("[%s] Enqueuing request", id)
	brokerRequests.WithLabelValues("client", server).Inc()
	select {
	case reqChan <- request:
		return respChan, nil
	case <-time.After(10 * time.Second):
		return nil, fmt.Errorf("%q doesn't appear to be running the relay client. Check that it's turned on, set up, and connected to the internet. (timeout waiting for relay client to accept request)", server)
	}
}

// GetRequest obtains a client's request for the server identifier. It blocks
// until a client makes a request.
func (r *broker) GetRequest(ctx context.Context, server string) (*pb.HttpRequest, error) {
	r.m.Lock()
	if r.req[server] == nil {
		// This happens when the relay-server started and a client connects before
		// the relay-client connected.
		r.req[server] = make(chan *pb.HttpRequest)
	}
	reqChan := r.req[server]
	r.m.Unlock()

	brokerRequests.WithLabelValues("server_request", server).Inc()
	select {
	case req := <-reqChan:
		brokerResponses.WithLabelValues("server_request", "ok", server).Inc()
		return req, nil
	case <-time.After(time.Second * 30):
		brokerResponses.WithLabelValues("server_request", "timeout", server).Inc()
		return nil, fmt.Errorf("No request received within timeout")
	case <-ctx.Done():
		return nil, fmt.Errorf("Server is restarting")
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
	case data := <-pr.requestStream:
		return data, true
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
	backendName := strings.SplitN(id, ":", 2)[0]
	r.m.Lock()
	pr := r.resp[id]
	if pr == nil {
		r.m.Unlock()
		brokerResponses.WithLabelValues("server_response", "invalid", backendName).Inc()
		return fmt.Errorf("Duplicate or invalid request ID %s", id)
	}
	if resp.GetEof() {
		delete(r.resp, id)
	} else {
		pr.lastActivity = time.Now()
	}
	duration := time.Since(pr.startTime).Seconds()
	pr.responseStream <- resp
	r.m.Unlock()
	brokerRequests.WithLabelValues("server_response", backendName).Inc()
	brokerResponseDurations.WithLabelValues("server_response", backendName).Observe(duration)
	log.Printf("[%s] Delivered response to client (%d bytes), elapsed %.3fs", id, len(resp.Body), duration)
	if resp.GetEof() {
		close(pr.responseStream)
	}
	brokerResponses.WithLabelValues("server_response", "ok", backendName).Inc()
	return nil
}

func (r *broker) ReapInactiveRequests(threshold time.Time) {
	r.m.Lock()
	for id, pr := range r.resp {
		if pr.lastActivity.Before(threshold) {
			log.Printf("[%s] Timeout on inactive request", id)
			defer close(pr.requestStream)
			defer close(pr.responseStream)
			// Amazingly, this is safe in Go: https://stackoverflow.com/questions/23229975/is-it-safe-to-remove-selected-keys-from-map-within-a-range-loop
			delete(r.resp, id)
		}
	}
	r.m.Unlock()
}
