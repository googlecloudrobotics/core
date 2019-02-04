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

// Package main runs a multiplexing HTTP relay server.
//
// It exists to make HTTP endpoints on robots accessible without a public
// endpoint. It binds to a public endpoint accessible by both client and
// backend and works together with a relay client that's colocated with the
// backend.
//
//  Private   Firewall   Internet Firewall     Private net
//                |                  |
//        client ---> relay server <--- relay client ---> backend
//                |                  |
//
// The relay server is multiplexing: It allows multiple relay clients to
// connect under unique names, each handling requests for a subpath of /client.
//
// Sequence of operations:
//   * Client makes request on /client/$foo/$request.
//   * Relay server assigns an ID and stores request (with path $request) in
//     memory. It keeps the client's request pending.
//   * Relay client requests /server/request?server=$foo
//   * Relay server responds with stored request.
//   * Relay client makes the stored request to backend.
//   * Backend replies.
//   * Relay client posts backend's reply to /server/response?id=$id.
//   * Relay server responds to client's request with backend's reply.
//
// The client side implementation is in ../http-relay-client.

package main

import (
	"encoding/hex"
	"flag"
	"fmt"
	"io/ioutil"
	"log"
	"math/rand"
	"net/http"
	"net/url"
	"strings"
	"time"

	pb "src/proto/http-relay"

	"github.com/golang/protobuf/proto"
	"github.com/prometheus/client_golang/prometheus/promhttp"
)

const (
	clientPrefix = "/client/"
)

var (
	port      = flag.Int("port", 80, "Port number to listen on")
	projectId = flag.String("project_id", "", "Cloud project for IAM checks")
)

func createId() string {
	u := make([]byte, 16)
	// err is documented as always nil
	rand.Read(u)
	return hex.EncodeToString(u)
}

func marshalHeader(h *http.Header) []*pb.HttpHeader {
	r := []*pb.HttpHeader{}
	for k, vs := range *h {
		for _, v := range vs {
			r = append(r, &pb.HttpHeader{Name: proto.String(k), Value: proto.String(v)})
		}
	}
	return r
}

func unmarshalHeader(w http.ResponseWriter, protoHeader []*pb.HttpHeader) {
	for _, h := range protoHeader {
		w.Header().Set(*h.Name, *h.Value)
	}
}

type server struct {
	b *broker
}

func newServer() *server {
	s := &server{b: newBroker()}
	go func() {
		for t := range time.Tick(10 * time.Second) {
			s.b.ReapInactiveRequests(t.Add(-30 * time.Second))
		}
	}()
	return s
}

// responseFilter enforces that there's at least one HttpResponse in the out
// channel and that the first and only the first response has headers.
func responseFilter(in <-chan *pb.HttpResponse, out chan<- *pb.HttpResponse) {
	firstMessage, more := <-in
	if !more {
		brokerResponses.WithLabelValues("client", "missing_message").Inc()
		out <- &pb.HttpResponse{
			StatusCode: proto.Int32(http.StatusInternalServerError),
			Body:       []byte(fmt.Sprintf("Received no response from relay client")),
		}
		close(out)
		return
	}
	if firstMessage.StatusCode == nil {
		brokerResponses.WithLabelValues("client", "missing_header").Inc()
		out <- &pb.HttpResponse{
			StatusCode: proto.Int32(http.StatusInternalServerError),
			Body:       []byte(fmt.Sprintf("Received no headers from relay client")),
		}
		close(out)
		// Flush remaining messages
		for range in {
		}
		return
	}
	out <- firstMessage
	for backendResp := range in {
		backendResp.StatusCode = nil
		backendResp.Header = nil
		brokerResponses.WithLabelValues("client", "ok").Inc()
		out <- backendResp
	}
	close(out)
}

func (s *server) client(w http.ResponseWriter, r *http.Request) {
	// After stripping, the path is "${SERVER_NAME}/${REQUEST}"
	pathParts := strings.SplitN(strings.TrimPrefix(r.URL.Path, clientPrefix), "/", 2)
	backendName := pathParts[0]

	log.Printf("Wrapping request for %s", r.URL.Path)
	body, err := ioutil.ReadAll(r.Body)
	if err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}
	backendUrl := url.URL{
		Scheme:   "http",
		Host:     "invalid",
		Path:     fmt.Sprintf("/%s", pathParts[1]),
		RawQuery: r.URL.RawQuery,
		Fragment: r.URL.Fragment,
	}

	id := createId()
	backendReq := &pb.HttpRequest{
		Id:     proto.String(id),
		Method: proto.String(r.Method),
		Url:    proto.String(backendUrl.String()),
		Header: marshalHeader(&r.Header),
		Body:   body,
	}

	backendRespChan, err := s.b.RelayRequest(backendName, backendReq)
	if err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}
	filtered := make(chan *pb.HttpResponse)
	go responseFilter(backendRespChan, filtered)
	for backendResp := range filtered {
		if backendResp.Header != nil {
			unmarshalHeader(w, backendResp.Header)
		}
		if backendResp.StatusCode != nil {
			w.WriteHeader(int(*backendResp.StatusCode))
		}
		w.Write([]byte(backendResp.Body))
		if flush, ok := w.(http.Flusher); ok {
			flush.Flush()
		}
	}

	log.Printf("Delivered response for request %s", id)
}

func (s *server) serverRequest(w http.ResponseWriter, r *http.Request) {
	server := r.URL.Query().Get("server")
	if server == "" {
		http.Error(w, "Missing server query parameter", http.StatusBadRequest)
		return
	}
	log.Printf("Relay client connected for server %s", server)

	request, err := s.b.GetRequest(server)
	if err != nil {
		log.Printf("Relay client got no request: %v", err)
		http.Error(w, err.Error(), http.StatusRequestTimeout)
		return
	}

	body, err := proto.Marshal(request)
	if err != nil {
		log.Printf("Failed to unmarshal request: %v", err)
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}

	w.Header().Set("Content-Type", "application/octet-data")
	w.Write(body)
	log.Printf("Relay client accepted request for %s", server)
}

func (s *server) serverResponse(w http.ResponseWriter, r *http.Request) {
	body, err := ioutil.ReadAll(r.Body)
	if err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}

	br := &pb.HttpResponse{}
	err = proto.Unmarshal(body, br)
	if err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}

	err = s.b.SendResponse(br)
	if err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}

	log.Printf("Client sent result for id %s", *br.Id)
	w.Header().Set("Content-Type", "text/plain")
	w.Write([]byte("ok"))
}

func main() {
	flag.Parse()

	server := newServer()

	http.HandleFunc("/healthz", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "text/plain")
		w.Write([]byte("ok"))
	})
	http.HandleFunc(clientPrefix, server.client)
	http.HandleFunc("/server/request", server.serverRequest)
	http.HandleFunc("/server/response", server.serverResponse)
	http.Handle("/metrics", promhttp.Handler())
	log.Fatal(http.ListenAndServe(fmt.Sprintf(":%d", *port), nil))
}
