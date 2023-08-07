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
	"context"
	"fmt"
	"net/url"
	"strings"
	"sync"
	"time"

	pb "github.com/googlecloudrobotics/core/src/proto/http-relay"
	"k8s.io/klog"

	"github.com/prometheus/client_golang/prometheus"
)

var (
	brokerRequests = prometheus.NewCounterVec(
		prometheus.CounterOpts{
			Name: "broker_requests",
			Help: "Number of requests to the broker",
		},
		[]string{"method", "backend", "request_path"},
	)
	brokerResponses = prometheus.NewCounterVec(
		prometheus.CounterOpts{
			Name: "broker_responses",
			Help: "Number of responses from the broker",
		},
		[]string{"method", "result", "backend", "request_path"},
	)
	brokerResponseDurations = prometheus.NewHistogramVec(
		prometheus.HistogramOpts{
			Name: "broker_responses_durations",
			Help: "Time from request to final response in s",
		},
		[]string{"method", "backend", "request_path"},
	)
	brokerBackendResponseDurations = prometheus.NewHistogramVec(
		prometheus.HistogramOpts{
			Name: "broker_backend_responses_durations",
			Help: "Time from backend request to final response in s",
		},
		[]string{"method", "backend", "request_path"},
	)
	brokerOverheadDurations = prometheus.NewHistogramVec(
		prometheus.HistogramOpts{
			Name: "broker_overhead_durations",
			Help: "Extra time spend between relay server and client in s",
		},
		[]string{"method", "backend", "request_path"},
	)
)

func init() {
	prometheus.MustRegister(brokerRequests)
	prometheus.MustRegister(brokerResponses)
	prometheus.MustRegister(brokerResponseDurations)
	prometheus.MustRegister(brokerBackendResponseDurations)
	prometheus.MustRegister(brokerOverheadDurations)
}

type pendingResponse struct {
	// This channel is used to communicate data between the backend and user-client for
	// bidirectional streaming connections.
	requestStream chan []byte

	// This channel is used to communicate data between the backend and user-client.
	// The user-client sends a hanging request to the relay-server which blocks until
	// data is received on the response channel.
	responseStream chan *pb.HttpResponse

	lastActivity time.Time
	// For diagnostics only.
	startTime   time.Time
	requestPath string
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
	targetUrl, err := url.Parse(*request.Url)
	if err != nil {
		return nil, fmt.Errorf("Failed to parse URL: %v", err)
	}

	r.m.Lock()
	if r.req[server] == nil {
		// This happens when the relay-client connects for the first time.
		r.req[server] = make(chan *pb.HttpRequest)
	}
	if r.resp[id] != nil {
		r.m.Unlock()
		return nil, fmt.Errorf("Multiple clients trying to handle request ID %s on server %s", id, server)
	}
	ts := time.Now()
	r.resp[id] = &pendingResponse{
		requestStream:  make(chan []byte),
		responseStream: make(chan *pb.HttpResponse),
		lastActivity:   ts,
		startTime:      ts,
		requestPath:    targetUrl.Path,
	}
	reqChan := r.req[server]
	respChan := r.resp[id].responseStream
	r.m.Unlock()

	klog.Infof("[%s] Enqueuing request", id)
	brokerRequests.WithLabelValues("client", server, targetUrl.Path).Inc()
	select {
	// This blocks until we get a free spot in the broker's request channel.
	case reqChan <- request:
		return respChan, nil
	case <-time.After(10 * time.Second):
		// This branch is triggered if the channel is not ready to consume the request
		// since it is still busy with handling a different request.
		return nil, fmt.Errorf("Cannot reach the client %q. Check that it's turned on, set up, and connected to the internet. (timeout waiting for relay client to accept request)", server)
	}
}

// GetRequest obtains a client's request for the server identifier. It blocks
// until a client makes a request.
func (r *broker) GetRequest(ctx context.Context, server, path string) (*pb.HttpRequest, error) {
	r.m.Lock()
	if r.req[server] == nil {
		// This happens when the relay-server started and a client connects before
		// the relay-client connected.
		r.req[server] = make(chan *pb.HttpRequest)
	}
	reqChan := r.req[server]
	r.m.Unlock()

	brokerRequests.WithLabelValues("server_request", server, path).Inc()
	select {
	case req := <-reqChan:
		brokerResponses.WithLabelValues("server_request", "ok", server, path).Inc()
		return req, nil
	case <-time.After(time.Second * 30):
		brokerResponses.WithLabelValues("server_request", "timeout", server, path).Inc()
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

// SendResponse delivers the HttpResponse to the user-client handler that created the
// request. It fails if and only if the request ID is not recognized.
func (r *broker) SendResponse(resp *pb.HttpResponse) error {
	id := *resp.Id
	backendName := strings.SplitN(id, ":", 2)[0]
	r.m.Lock()
	pr := r.resp[id]
	if pr == nil {
		r.m.Unlock()
		brokerResponses.WithLabelValues("server_response", "invalid", backendName, "").Inc()
		return fmt.Errorf("Duplicate or invalid request ID %s", id)
	}
	if resp.GetEof() {
		delete(r.resp, id)
	} else {
		pr.lastActivity = time.Now()
	}
	duration := time.Since(pr.startTime).Seconds()

	// Writing to this channel will notify consumers which are waiting for data
	// on the channel returned by RelayRequest().
	pr.responseStream <- resp

	r.m.Unlock()
	brokerRequests.WithLabelValues("server_response", backendName, pr.requestPath).Inc()
	brokerResponseDurations.WithLabelValues("server_response", backendName, pr.requestPath).Observe(duration)
	if resp.GetEof() {
		close(pr.responseStream)
		backendDuration := (time.Duration(resp.GetBackendDurationMs()) * time.Millisecond).Seconds()
		backendDurStr := "N/A"
		if backendDuration > 0.0 {
			brokerBackendResponseDurations.WithLabelValues("server_response", backendName, pr.requestPath).Observe(backendDuration)
			brokerOverheadDurations.WithLabelValues("server_response", backendName, pr.requestPath).Observe(duration - backendDuration)
			backendDurStr = fmt.Sprintf("%.3fs", backendDuration)
		}
		klog.Infof("[%s] Delivered final response to client (%d bytes), elapsed on server=%.3fs, backend=%s", id, len(resp.Body), duration, backendDurStr)
	} else {
		klog.Infof("[%s] Delivered response to client (%d bytes), elapsed on server=%.3fs", id, len(resp.Body), duration)
	}
	brokerResponses.WithLabelValues("server_response", "ok", backendName, pr.requestPath).Inc()
	return nil
}

func (r *broker) ReapInactiveRequests(threshold time.Time) {
	r.m.Lock()
	for id, pr := range r.resp {
		if pr.lastActivity.Before(threshold) {
			klog.Infof("[%s] Timeout on inactive request", id)
			defer close(pr.requestStream)
			defer close(pr.responseStream)
			// Amazingly, this is safe in Go: https://stackoverflow.com/questions/23229975/is-it-safe-to-remove-selected-keys-from-map-within-a-range-loop
			delete(r.resp, id)
		}
	}
	r.m.Unlock()
}
