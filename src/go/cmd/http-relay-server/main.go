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

// Package main runs a multiplexing HTTP relay server.
//
// It exists to make HTTP endpoints on robots accessible without a public
// endpoint. It binds to a public endpoint accessible by both user-client and
// backend and works together with a relay-client that's colocated with the
// backend.
//
//	  lan        |    internet      |               lan
//	             |                  |
//	user-client ---> relay server <--- relay-client ---> backend
//	             |                  |
//	         firewall           firewall
//
// The relay server is multiplexing: It allows multiple relay-clients to
// connect under unique names, each handling requests for a subpath of /client.
// Alternatively (e.g. for grpc conenctions) the backend can be selected by
// omitting the client prefix and passing an `X-Server-Name` header.
//
// Sequence of operations:
//   - Web-client makes request on /client/$foo/$request.
//   - Relay server assigns an ID and stores request (with path $request) in
//     memory. It keeps the user-client's request pending.
//   - Relay-client requests /server/request?server=$foo
//   - Relay server responds with stored request (or timeout if no request comes
//     in within the next 30 sec).
//   - Relay-client makes the stored request to backend.
//   - Backend replies.
//   - Relay-client posts backend's reply to /server/response.
//   - Relay server responds to client's request with backend's reply.
//
// For some requests (eg kubectl exec), the backend responds with
// 101 Switching Protocols, resulting in the following operations.
//   - Relay server responds to client's request with backend's 101 reply.
//   - Client sends bytes from stdin to the relay server.
//   - Relay-client requests /server/requeststream?id=$id.
//   - Relay server responds with stdin bytes from client.
//   - Relay-client sends stdin bytes to backend.
//   - Backend sends stdout bytes to relay-client.
//   - Relay-client posts stdout bytes to /server/response.
//   - Relay server sends stdout bytes to the client.
//
// This simplified graphic shows the back-and-forth for an `exec` request:
//
//	user-client ---> relay server <--- relay-client ---> backend
//	      .     |        .         |       .               .
//	      . -POST /exec->.         |       .               .
//	      .     |        . <-GET /request- .               .
//	      .     |        . ---- exec ----> .               .
//	      .     |        .         |       . -POST /exec-> .
//	      .     |        .         |       . <--- 101 ---- .
//	      .     |        .<-POST /response-.               .
//	      . <-- 101 ---- .         |       .               .
//	      . -- stdin --> .         |       .               .
//	      .     |        .<-POST /request- .               .
//	      .     |        .        stream   .               .
//	      .     |        . ---- stdin ---> .               .
//	      .     |        .         |       . --- stdin --> .
//	      .     |        .         |       . <-- stdout--- .
//	      .     |        .<-POST /response-.               .
//	      . <- stdout -- .         |       .               .
//	      .     |        .         |       .               .
//
// The relay-client side implementation is in ../http-relay-client.
package main

import (
	"context"
	"flag"
	"fmt"
	"io/ioutil"
	"log"
	"net"
	"net/http"
	"net/http/httputil"
	"os"
	"os/signal"
	"syscall"
	"time"

	pb "github.com/googlecloudrobotics/core/src/proto/http-relay"
	"github.com/prometheus/client_golang/prometheus/promhttp"
	"google.golang.org/protobuf/proto"

	"contrib.go.opencensus.io/exporter/stackdriver"
	"go.opencensus.io/plugin/ochttp"
	"go.opencensus.io/plugin/ochttp/propagation/tracecontext"
	"go.opencensus.io/trace"
	"golang.org/x/net/http2"
	"golang.org/x/net/http2/h2c"
	"golang.org/x/sync/errgroup"
)

const (
	inactiveRequestTimeout = 60 * time.Second
	// Time to wait for requests to complete before calling panic(). This should
	// be less that the kubelet's timeout (30s by default) so that we can print
	// a stack trace and debug what is still running.
	cleanShutdownTimeout = 20 * time.Second
	// Print more detailed logs when enabled.
	debugLogs = false
)

var (
	port      = flag.Int("port", 80, "Port number to listen on")
	blockSize = flag.Int("block_size", 10*1024,
		"Size of i/o buffer in bytes")
	stackdriverProjectID = flag.String("trace-stackdriver-project-id", "",
		"If not empty, traces will be uploaded to this Google Cloud Project.")
)

func waitForFirstResponse(backendCtx backendContext, backendRespCh <-chan *pb.HttpResponse) (*pb.HttpResponse, error) {
	firstResponse, more := <-backendRespCh
	if !more {
		// There needs to be at least a single response from the backend client.
		brokerResponses.WithLabelValues("client", "missing_message", backendCtx.ServerName, backendCtx.Path).Inc()
		return nil, fmt.Errorf("Timeout after %v, either the backend request took too long or the relay client died.", inactiveRequestTimeout)
	}
	return firstResponse, nil
}

type server struct {
	b *broker
}

func newServer() *server {
	s := &server{b: newBroker()}
	go func() {
		for t := range time.Tick(10 * time.Second) {
			s.b.ReapInactiveRequests(t.Add(-1 * inactiveRequestTimeout))
		}
	}()
	return s
}

func (s *server) health(w http.ResponseWriter, r *http.Request) {
	if err := s.b.Healthy(); err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}
	w.Header().Set("Content-Type", "text/plain")
	w.Write([]byte("ok"))
}

func (s *server) readRequestBody(ctx context.Context, r *http.Request) ([]byte, error) {
	_, span := trace.StartSpan(ctx, "Read request body")
	addServiceName(span)
	defer span.End()
	return ioutil.ReadAll(r.Body)
}

func (s *server) relayRequest(ctx context.Context, backendCtx backendContext, request *pb.HttpRequest) (<-chan *pb.HttpResponse, error) {
	_, span := trace.StartSpan(ctx, "Schedule request for pickup")
	addServiceName(span)
	defer span.End()

	backendRespChan, err := s.b.RelayRequest(backendCtx.ServerName, request)
	if err != nil {
		return nil, err
	}
	return backendRespChan, nil
}

// This function is used to handle requests by the user-client.
// This is e.g. a browser request.
func (s *server) userClientRequest(w http.ResponseWriter, r *http.Request) {
	f := &tracecontext.HTTPFormat{}
	var span *trace.Span
	ctx := r.Context()
	if sctx, ok := f.SpanContextFromRequest(r); ok {
		ctx, span = trace.StartSpanWithRemoteParent(ctx, "Received user client request "+r.URL.Path, sctx)
	} else {
		ctx, span = trace.StartSpan(ctx, "Received user client request "+r.URL.Path)
	}
	addServiceName(span)
	// Embedding the span in the request ensures that the server side spans are correctly
	// nested.
	// Note: We are overwriting the previous one with the current one which is not a
	// problem since we have already read the previous one.
	f.SpanContextToRequest(span.SpanContext(), r)
	// We can actually defer span.end() since this function will wait until a response from
	// a server is being received.
	defer span.End()

	if debugLogs {
		dump, _ := httputil.DumpRequest(r, false)
		log.Printf("%s", dump)
	}

	backendCtx, err := newBackendContext(r)
	if err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}

	body, err := s.readRequestBody(ctx, r)
	if err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}

	backendReq := createBackendRequest(*backendCtx, r, body)

	// Pipe a request into the request channel to it get polled by the relay client.
	// Then return the response channel, so we can pass it on and wait on a response
	// from the relay-client.
	backendRespChan, err := s.relayRequest(ctx, *backendCtx, backendReq)
	if err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}

	firstResponse, err := waitForFirstResponse(*backendCtx, backendRespChan)
	if err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}

	if *firstResponse.StatusCode == http.StatusSwitchingProtocols {
		span.AddAttributes(trace.StringAttribute("notes", "Received 101 switching protocols."))
		s.b.m.Lock()
		bodyStreamToBackend := s.b.resp[backendCtx.Id].requestStream
		s.b.m.Unlock()
		if bodyStreamToBackend != nil {
			websocketRelayResponses(*backendCtx, w, firstResponse, backendRespChan, bodyStreamToBackend)
		}
	} else {
		httpRelayResponses(ctx, *backendCtx, w, firstResponse, backendRespChan)
	}
}

// relay-client pulls a request
func (s *server) serverRequest(w http.ResponseWriter, r *http.Request) {
	server := r.URL.Query().Get("server")
	if server == "" {
		http.Error(w, "Missing server query parameter", http.StatusBadRequest)
		return
	}
	log.Printf("[%s] Relay client connected", server)

	// Get pending request from client and sent as a reply to the relay-client.
	request, err := s.b.GetRequest(r.Context(), server, r.URL.Path)
	if err != nil {
		log.Printf("[%s] Relay client got no request: %v", server, err)
		http.Error(w, err.Error(), http.StatusRequestTimeout)
		return
	}

	body, err := proto.Marshal(request)
	if err != nil {
		log.Printf("[%s] Failed to marshal request: %v", *request.Id, err)
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}

	w.Header().Set("Content-Type", "application/vnd.google.protobuf;proto=cloudrobotics.http_relay.v1alpha1.HttpRequest")
	w.Write(body)
	log.Printf("[%s] Relay client accepted request", *request.Id)
}

func (s *server) serverRequestStream(w http.ResponseWriter, r *http.Request) {
	id := r.URL.Query().Get("id")
	if id == "" {
		http.Error(w, "Missing id query parameter", http.StatusBadRequest)
		return
	}
	data, ok := s.b.GetRequestStream(id)
	if !ok {
		// Using the 410 Gone error tells the relay client that this request
		// has completed.
		http.Error(w, "No ongoing request with id "+id, http.StatusGone)
		return
	}

	w.Header().Set("Content-Type", "application/octet-data")
	w.Write(data)
	log.Printf("[%s] Relay client pulled streamed request data of %d bytes", id, len(data))
}

// This function receives the response from the relay-client after it processed
// the initial request in the backend.
// The response is stored in the response channel through which the data is relayed
// to the initial requester.
func (s *server) serverResponse(w http.ResponseWriter, r *http.Request) {
	body, err := ioutil.ReadAll(r.Body)
	if err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}

	br := &pb.HttpResponse{}
	if err = proto.Unmarshal(body, br); err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}

	// Send the response to the actual user-client using our broker.
	if err = s.b.SendResponse(br); err != nil {
		// SendResponse fails if and only if the request ID is bad.
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}
	// Respond to the relay-client and notify on successful propagation of
	// the backend response.
	w.Header().Set("Content-Type", "text/plain")
	w.Write([]byte("ok"))

	log.Printf("[%s] Relay client sent response", *br.Id)
}

func main() {
	flag.Parse()
	log.SetFlags(log.LstdFlags | log.Lmicroseconds)

	if *stackdriverProjectID != "" {
		sd, err := stackdriver.NewExporter(stackdriver.Options{
			ProjectID: *stackdriverProjectID,
		})
		if err != nil {
			log.Fatalf("Failed to create the Stackdriver exporter for project '%s': %v", *stackdriverProjectID, err)
		} else {
			trace.RegisterExporter(sd)
			defer sd.Flush()
		}
	}

	server := newServer()
	h2s := &http2.Server{}

	h := http.NewServeMux()
	h.HandleFunc("/healthz", server.health)
	h.HandleFunc("/", server.userClientRequest)
	h.HandleFunc("/server/request", server.serverRequest)
	h.HandleFunc("/server/requeststream", server.serverRequestStream)
	h.HandleFunc("/server/response", server.serverResponse)
	h.Handle("/metrics", promhttp.Handler())

	// This context will be terminated we get SIGTERM from Kubernetes. We need
	// some boilerplate to make this terminate the HTTP server and the ongoing
	// request contexts. This is based on:
	// https://www.rudderstack.com/blog/implementing-graceful-shutdown-in-go/#:~:text=Canceling%20long%20running%20requests
	mainCtx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	h2h := h2c.NewHandler(h, h2s)
	och := &ochttp.Handler{
		Handler: h2h,
	}
	h1s := &http.Server{
		Addr:    fmt.Sprintf(":%d", *port),
		Handler: och,
		BaseContext: func(l net.Listener) context.Context {
			log.Printf("Relay server listening on: 127.0.0.1:%d", l.Addr().(*net.TCPAddr).Port)
			return mainCtx
		},
	}
	// Wait for the server to terminate, either because it failed to create a
	// listener, or because we got SIGTERM.
	g, gCtx := errgroup.WithContext(mainCtx)
	g.Go(func() error {
		if err := h1s.ListenAndServe(); err != http.ErrServerClosed {
			return err
		}
		// ErrServerClosed follows SIGTERM which is normal when updating the
		// server.
		return nil
	})
	g.Go(func() error {
		<-gCtx.Done()
		ctx, cancel := context.WithTimeout(context.Background(), cleanShutdownTimeout)
		defer cancel()
		return h1s.Shutdown(ctx)
	})

	if err := g.Wait(); err != nil {
		// SIGTERM indicates either a normal shutdown (eg pod update, node pool
		// update) or a failed liveness check (eg broker deadlock), we can't
		// easily tell. We panic to help debugging: if the environment sets
		// GOTRACEBACK=all they will see stacktraces after the panic.
		log.Panicf("Server terminated abnormally: %s", err)
	}
}
