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
// endpoint. It binds to a public endpoint accessible by both client and
// backend and works together with a relay client that's colocated with the
// backend.
//
//	  lan   |    internet      |               lan
//	        |                  |
//	client ---> relay server <--- relay client ---> backend
//	        |                  |
//	     firewall           firewall
//
// The relay server is multiplexing: It allows multiple relay clients to
// connect under unique names, each handling requests for a subpath of /client.
// Alternatively (e.g. for grpc conenctions) the backend can be selected by
// omitting the client prefix and passing an `X-Server-Name` header.
//
// Sequence of operations:
//   - Client makes request on /client/$foo/$request.
//   - Relay server assigns an ID and stores request (with path $request) in
//     memory. It keeps the client's request pending.
//   - Relay client requests /server/request?server=$foo
//   - Relay server responds with stored request (or timeout if no request comes
//     in within the next 30 sec).
//   - Relay client makes the stored request to backend.
//   - Backend replies.
//   - Relay client posts backend's reply to /server/response.
//   - Relay server responds to client's request with backend's reply.
//
// For some requests (eg kubectl exec), the backend responds with
// 101 Switching Protocols, resulting in the following operations.
//   - Relay server responds to client's request with backend's 101 reply.
//   - Client sends bytes from stdin to the relay server.
//   - Relay client requests /server/requeststream?id=$id.
//   - Relay server responds with stdin bytes from client.
//   - Relay client sends stdin bytes to backend.
//   - Backend sends stdout bytes to relay client.
//   - Relay client posts stdout bytes to /server/response.
//   - Relay server sends stdout bytes to the client.
//
// This simplified graphic shows the back-and-forth for an `exec` request:
//
//	client ---> relay server <--- relay client ---> backend
//	  .     |        .         |       .               .
//	  . -POST /exec->.         |       .               .
//	  .     |        . <-GET /request- .               .
//	  .     |        . ---- exec ----> .               .
//	  .     |        .         |       . -POST /exec-> .
//	  .     |        .         |       . <--- 101 ---- .
//	  .     |        .<-POST /response-.               .
//	  . <-- 101 ---- .         |       .               .
//	  . -- stdin --> .         |       .               .
//	  .     |        .<-POST /request- .               .
//	  .     |        .        stream   .               .
//	  .     |        . ---- stdin ---> .               .
//	  .     |        .         |       . --- stdin --> .
//	  .     |        .         |       . <-- stdout--- .
//	  .     |        .<-POST /response-.               .
//	  . <- stdout -- .         |       .               .
//	  .     |        .         |       .               .
//
// The client side implementation is in ../http-relay-client.
package main

import (
	"context"
	"encoding/hex"
	"flag"
	"fmt"
	"io/ioutil"
	"log"
	"math/rand"
	"net"
	"net/http"
	"net/http/httputil"
	"net/url"
	"os"
	"os/signal"
	"strings"
	"syscall"
	"time"

	_ "net/http/pprof"

	pb "github.com/googlecloudrobotics/core/src/proto/http-relay"
	"github.com/prometheus/client_golang/prometheus/promhttp"
	"google.golang.org/protobuf/proto"

	"golang.org/x/net/http2"
	"golang.org/x/net/http2/h2c"
	"golang.org/x/sync/errgroup"
)

const (
	clientPrefix           = "/client/"
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
		w.Header().Add(*h.Name, *h.Value)
	}
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

// responseFilter enforces that there's at least one HttpResponse in the out
// channel and that the first response has a status code. From the reposnses it
// extracts and return headers, trailers, status-code and body data.
func responseFilter(in <-chan *pb.HttpResponse, id, path string) ([]*pb.HttpHeader, []*pb.HttpHeader, int, <-chan []byte) {
	body := make(chan []byte, 1)
	firstMessage, more := <-in
	if !more {
		brokerResponses.WithLabelValues("client", "missing_message", id, path).Inc()
		body <- []byte(fmt.Sprintf("Timeout after %v, either the backend request took too long or the relay client died", inactiveRequestTimeout))
		close(body)
		return nil, nil, http.StatusInternalServerError, body
	}
	if firstMessage.StatusCode == nil {
		brokerResponses.WithLabelValues("client", "missing_header", id, path).Inc()
		body <- []byte("Received no header from relay client")
		close(body)
		// Flush remaining messages
		for range in {
		}
		return nil, nil, http.StatusInternalServerError, body
	}
	body <- []byte(firstMessage.Body)
	lastMessage := firstMessage
	go func() {
		for backendResp := range in {
			brokerResponses.WithLabelValues("client", "ok", id, path).Inc()
			body <- []byte(backendResp.Body)
			lastMessage = backendResp
		}
		close(body)
	}()
	return firstMessage.Header, lastMessage.Trailer, int(*firstMessage.StatusCode), body
}

// bidirectionalStream handles a 101 Switching Protocols response from the
// backend, by "hijacking" to get a bidirectional connection to the client,
// and streaming data between client and broker/relay client.
func (s *server) bidirectionalStream(w http.ResponseWriter, id string, response <-chan []byte) {
	hj, ok := w.(http.Hijacker)
	if !ok {
		http.Error(w, "Backend returned 101 Switching Protocols, which is not supported by the relay server", http.StatusInternalServerError)
		return
	}
	w.WriteHeader(http.StatusSwitchingProtocols)
	conn, bufrw, err := hj.Hijack()
	if err != nil {
		// After a failed hijack, the connection is in an unknown state and
		// we can't report an error to the client.
		log.Printf("[%s] Failed to hijack connection after 101: %v", id, err)
		return
	}
	log.Printf("[%s] Switched protocols", id)
	defer conn.Close()

	go func() {
		// This goroutine handles the request stream from client to backend.
		log.Printf("[%s] Trying to read from bidi-stream", id)
		for {
			// This must be a new buffer each time, as the channel is not making a copy
			bytes := make([]byte, *blockSize)
			// Here we get the client stream (e.g. kubectl or k9s)
			n, err := bufrw.Read(bytes)
			if err != nil {
				// TODO(https://github.com/golang/go/issues/4373): in Go 1.13,
				// we may be able to suppress the "read from closed connection" better.
				if strings.Contains(err.Error(), "use of closed network connection") {
					// Request ended and connection closed by HTTP server.
					log.Printf("[%s] End of bidi-stream stream (closed socket)", id)
				} else {
					// Connection has unexpectedly failed for some other reason.
					log.Printf("[%s] Error reading from bidi-stream: %v", id, err)
				}
				return
			}
			log.Printf("[%s] Read %d bytes from bidi-stream", id, n)
			if ok = s.b.PutRequestStream(id, bytes[:n]); !ok {
				log.Printf("[%s] End of bidi-stream stream", id)
				return
			}
			log.Printf("[%s] Uploaded %d bytes from bidi-stream", id, n)
		}
	}()

	numBytes := 0
	for bytes := range response {
		// TODO(b/130706300): detect dropped connection and end request in broker
		_, _ = bufrw.Write(bytes)
		bufrw.Flush()
		numBytes += len(bytes)
	}
	log.Printf("[%s] Wrote %d response bytes to bidi-stream", id, numBytes)
}

// client sent a request.
func (s *server) client(w http.ResponseWriter, r *http.Request) {
	if debugLogs {
		dump, _ := httputil.DumpRequest(r, false)
		log.Printf("%s", dump)
	}

	var backendName, path string
	if strings.HasPrefix(r.URL.Path, clientPrefix) {
		// After stripping, the path is "${SERVER_NAME}/${REQUEST}"
		pathParts := strings.SplitN(strings.TrimPrefix(r.URL.Path, clientPrefix), "/", 2)
		backendName = pathParts[0]
		if backendName == "" {
			http.Error(w, "Request path too short: missing remote server identifier", http.StatusBadRequest)
			return
		}
		path = "/"
		if len(pathParts) > 1 {
			path += pathParts[1]
		}
	} else {
		// Requests without the /client/ prefix are gRPC requests. The backend is
		// identified by "X-Server-Name" header.
		headers, ok := r.Header["X-Server-Name"]
		if !ok {
			http.Error(w, "Request without required header: \"X-Server-Name\"", http.StatusBadRequest)
			return
		}
		backendName = headers[0]
		path = r.URL.Path
	}

	id := backendName + ":" + createId()
	log.Printf("[%s] Wrapping request for %s%s", id, backendName, path)

	body, err := ioutil.ReadAll(r.Body)
	if err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}
	log.Printf("[%s] Read %d bytes from client", id, len(body))
	backendUrl := url.URL{
		Scheme:   "http",
		Host:     "invalid",
		Path:     path,
		RawQuery: r.URL.RawQuery,
		Fragment: r.URL.Fragment,
	}
	backendReq := &pb.HttpRequest{
		Id:     proto.String(id),
		Method: proto.String(r.Method),
		Host:   proto.String(r.Host),
		Url:    proto.String(backendUrl.String()),
		Header: marshalHeader(&r.Header),
		Body:   body,
	}

	backendRespChan, err := s.b.RelayRequest(backendName, backendReq)
	if err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}
	header, trailer, status, response := responseFilter(backendRespChan, backendName, path)
	if header != nil {
		unmarshalHeader(w, header)
	}

	if status == http.StatusSwitchingProtocols {
		// Note: call s.bidirectionalStream before w.WriteHeader so that
		// bidirectionalStream can set the status on error.
		s.bidirectionalStream(w, id, response)
		return
	}

	w.WriteHeader(status)
	numBytes := 0
	for bytes := range response {
		// TODO(b/130706300): detect dropped connection and end request in broker
		_, _ = w.Write(bytes)
		if flush, ok := w.(http.Flusher); ok {
			flush.Flush()
		}
		numBytes += len(bytes)
	}
	// TODO(ensonic): open questions:
	// - can we do this less hacky? (see unmarshalHeader() above)
	// - why do we not always get them as trailers?
	for _, h := range header {
		if strings.HasPrefix(*h.Name, "Grpc-") {
			w.Header().Add(http.TrailerPrefix+*h.Name, *h.Value)
			log.Printf("[%s] Adding trailer from header: %q:%q", id, *h.Name, *h.Value)
		}
	}
	if trailer != nil {
		for _, h := range trailer {
			w.Header().Add(http.TrailerPrefix+*h.Name, *h.Value)
			log.Printf("[%s] Adding real trailer: %q:%q", id, *h.Name, *h.Value)
		}
	}

	log.Printf("[%s] Wrote %d response bytes to request", id, numBytes)
}

// relay-client sent a request.
func (s *server) serverRequest(w http.ResponseWriter, r *http.Request) {
	server := r.URL.Query().Get("server")
	if server == "" {
		http.Error(w, "Missing server query parameter", http.StatusBadRequest)
		return
	}
	log.Printf("[%s] Relay client connected", server)

	// get pending request from client and sent as a reply to the relay-client
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

	if err = s.b.SendResponse(br); err != nil {
		// SendResponse fails if and only if the request ID is bad.
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}
	w.Header().Set("Content-Type", "text/plain")
	w.Write([]byte("ok"))
	log.Printf("[%s] Relay client sent response", *br.Id)
}

func main() {
	flag.Parse()
	log.SetFlags(log.LstdFlags | log.Lmicroseconds)

	// for pprof
	go http.ListenAndServe(":8084", nil)

	server := newServer()
	h2s := &http2.Server{}

	h := http.NewServeMux()
	h.HandleFunc("/healthz", server.health)
	h.HandleFunc("/", server.client)
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
	h1s := &http.Server{
		Addr:    fmt.Sprintf(":%d", *port),
		Handler: h2c.NewHandler(h, h2s),
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
