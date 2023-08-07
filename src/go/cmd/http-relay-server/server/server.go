package server

import (
	"context"
	"encoding/hex"
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

	pb "github.com/googlecloudrobotics/core/src/proto/http-relay"
	"github.com/prometheus/client_golang/prometheus/promhttp"
	"google.golang.org/protobuf/proto"

	"go.opencensus.io/plugin/ochttp"
	"go.opencensus.io/plugin/ochttp/propagation/tracecontext"
	"go.opencensus.io/trace"
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

type Server struct {
	port      int // Port number to listen on
	blockSize int // Size of i/o buffer in bytes
	b         *broker
}

func NewServer() *Server {
	s := &Server{
		port:      80,
		blockSize: 10 * 1024,
		b:         newBroker(),
	}
	go func() {
		for t := range time.Tick(10 * time.Second) {
			s.b.ReapInactiveRequests(t.Add(-1 * inactiveRequestTimeout))
		}
	}()
	return s
}

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

func addServiceName(span *trace.Span) {
	relayServerAttr := trace.StringAttribute("service.name", "http-relay-server")
	span.AddAttributes(relayServerAttr)
}

func extractBackendNameAndPath(r *http.Request) (backendName string, path string, err error) {
	if strings.HasPrefix(r.URL.Path, clientPrefix) {
		// After stripping, the path is "${SERVER_NAME}/${REQUEST}"
		pathParts := strings.SplitN(strings.TrimPrefix(r.URL.Path, clientPrefix), "/", 2)
		backendName = pathParts[0]
		if backendName == "" {
			err = fmt.Errorf("Request path too short: missing remote server identifier")
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
			err = fmt.Errorf("Request without required header: \"X-Server-Name\"")
			return
		}
		backendName = headers[0]
		path = r.URL.Path
	}
	return
}

// responseFilter enforces that there's at least one HttpResponse in the out
// channel and that the first response has a status code. From the reposnses it
// extracts and return headers, trailers, status-code and body data.
func responseFilter(backendCtx backendContext, in <-chan *pb.HttpResponse) ([]*pb.HttpHeader, []*pb.HttpHeader, int, <-chan []byte) {
	body := make(chan []byte, 1)
	firstMessage, more := <-in
	if !more {
		brokerResponses.WithLabelValues("client", "missing_message", backendCtx.ServerName, backendCtx.Path).Inc()
		body <- []byte(fmt.Sprintf("Timeout after %v, either the backend request took too long or the relay client died", inactiveRequestTimeout))
		close(body)
		return nil, nil, http.StatusInternalServerError, body
	}
	if firstMessage.StatusCode == nil {
		brokerResponses.WithLabelValues("client", "missing_header", backendCtx.ServerName, backendCtx.Path).Inc()
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
			brokerResponses.WithLabelValues("client", "ok", backendCtx.ServerName, backendCtx.Path).Inc()
			body <- []byte(backendResp.Body)
			lastMessage = backendResp
		}
		close(body)
	}()
	// TODO(haukeheibel): We are returning before the go-routine above finishes. How is
	// lastMessage.Trailer different from firstMessage.Trailer?
	return firstMessage.Header, lastMessage.Trailer, int(*firstMessage.StatusCode), body
}

type backendContext struct {
	Id         string
	ServerName string
	Path       string
}

func newBackendContext(r *http.Request) (*backendContext, error) {
	serverName, path, err := extractBackendNameAndPath(r)
	if err != nil {
		return nil, err
	}
	return &backendContext{
		Id:         serverName + ":" + createId(),
		ServerName: serverName,
		Path:       path,
	}, nil
}

func (s *Server) health(w http.ResponseWriter, r *http.Request) {
	if err := s.b.Healthy(); err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}
	w.Header().Set("Content-Type", "text/plain")
	w.Write([]byte("ok"))
}

// bidirectionalStream handles a 101 Switching Protocols response from the
// backend, by "hijacking" to get a bidirectional connection to the client,
// and streaming data between client and broker/relay client.
func (s *Server) bidirectionalStream(backendCtx backendContext, w http.ResponseWriter, response <-chan []byte) {
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
		log.Printf("[%s] Failed to hijack connection after 101: %v", backendCtx.Id, err)
		return
	}
	log.Printf("[%s] Switched protocols", backendCtx.Id)
	defer conn.Close()

	go func() {
		// This goroutine handles the request stream from client to backend.
		log.Printf("[%s] Trying to read from bidi-stream", backendCtx.Id)
		for {
			// This must be a new buffer each time, as the channel is not making a copy
			bytes := make([]byte, s.blockSize)
			// Here we get the client stream (e.g. kubectl or k9s)
			n, err := bufrw.Read(bytes)
			if err != nil {
				// TODO(https://github.com/golang/go/issues/4373): in Go 1.13,
				// we may be able to suppress the "read from closed connection" better.
				if strings.Contains(err.Error(), "use of closed network connection") {
					// Request ended and connection closed by HTTP server.
					log.Printf("[%s] End of bidi-stream stream (closed socket)", backendCtx.Id)
				} else {
					// Connection has unexpectedly failed for some other reason.
					log.Printf("[%s] Error reading from bidi-stream: %v", backendCtx.Id, err)
				}
				return
			}
			log.Printf("[%s] Read %d bytes from bidi-stream", backendCtx.Id, n)
			if ok = s.b.PutRequestStream(backendCtx.Id, bytes[:n]); !ok {
				log.Printf("[%s] End of bidi-stream stream", backendCtx.Id)
				return
			}
			log.Printf("[%s] Uploaded %d bytes from bidi-stream", backendCtx.Id, n)
		}
	}()

	numBytes := 0
	for bytes := range response {
		// TODO(b/130706300): detect dropped connection and end request in broker
		_, _ = bufrw.Write(bytes)
		bufrw.Flush()
		numBytes += len(bytes)
	}
	log.Printf("[%s] Wrote %d response bytes to bidi-stream", backendCtx.Id, numBytes)
}

func (s *Server) readRequestBody(ctx context.Context, r *http.Request) ([]byte, error) {
	_, span := trace.StartSpan(ctx, "Read request body")
	addServiceName(span)
	defer span.End()
	return ioutil.ReadAll(r.Body)
}

func (s *Server) createBackendRequest(backendCtx backendContext, r *http.Request, body []byte) *pb.HttpRequest {
	backendUrl := url.URL{
		Scheme:   "http",
		Host:     "invalid",
		Path:     backendCtx.Path,
		RawQuery: r.URL.RawQuery,
		Fragment: r.URL.Fragment,
	}

	backendReq := &pb.HttpRequest{
		Id:     proto.String(backendCtx.Id),
		Method: proto.String(r.Method),
		Host:   proto.String(r.Host),
		Url:    proto.String(backendUrl.String()),
		Header: marshalHeader(&r.Header),
		Body:   body,
	}

	return backendReq
}

func (s *Server) relayRequest(ctx context.Context, backendCtx backendContext, request *pb.HttpRequest) (<-chan *pb.HttpResponse, error) {
	_, span := trace.StartSpan(ctx, "Schedule request for pickup")
	addServiceName(span)
	defer span.End()

	backendRespChan, err := s.b.RelayRequest(backendCtx.ServerName, request)
	if err != nil {
		return nil, err
	}
	return backendRespChan, nil
}

func (s *Server) waitForFirstResponseAndHandleSwitching(ctx context.Context, backendCtx backendContext, w http.ResponseWriter, backendRespChan <-chan *pb.HttpResponse) ([]*pb.HttpHeader, []*pb.HttpHeader, <-chan []byte, bool) {
	_, span := trace.StartSpan(ctx, "Waiting for first response")
	addServiceName(span)
	defer span.End()

	header, trailer, status, backendRespBodyChan := responseFilter(backendCtx, backendRespChan)
	if header != nil {
		unmarshalHeader(w, header)
	}

	if status == http.StatusSwitchingProtocols {
		span.AddAttributes(trace.StringAttribute("notes", "Received 101 switching protocols."))
		// Note: call s.bidirectionalStream before w.WriteHeader so that
		// bidirectionalStream can set the status on error.
		// TODO(haukeheibel): I don't get this comment. We never write the
		// header and just return.
		s.bidirectionalStream(backendCtx, w, backendRespBodyChan)
		return nil, nil, nil, true
	}

	w.WriteHeader(status)

	return header, trailer, backendRespBodyChan, false
}

// This function is used to handle requests by the user-client.
// This is e.g. a browser request.
func (s *Server) userClientRequest(w http.ResponseWriter, r *http.Request) {
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

	backendReq := s.createBackendRequest(*backendCtx, r, body)

	// Pipe a request into the request channel to it get polled by the relay client.
	// Then return the response channel, so we can pass it on and wait on a response
	// from the relay-client.
	backendRespChan, err := s.relayRequest(ctx, *backendCtx, backendReq)
	if err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}

	header, trailer, backendRespBodyChannel, done := s.waitForFirstResponseAndHandleSwitching(ctx, *backendCtx, w, backendRespChan)
	if done {
		return
	}

	_, forwardingResponseSpan := trace.StartSpan(ctx, "Forwarding backend response to user-client")
	addServiceName(forwardingResponseSpan)
	defer forwardingResponseSpan.End()

	// This code here will block until we have actually received a response from the backend,
	// i.e. this will block until
	numBytes := 0
	for bytes := range backendRespBodyChannel {
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
			log.Printf("[%s] Adding trailer from header: %q:%q", backendCtx.Id, *h.Name, *h.Value)
		}
	}
	if trailer != nil {
		for _, h := range trailer {
			w.Header().Add(http.TrailerPrefix+*h.Name, *h.Value)
			log.Printf("[%s] Adding real trailer: %q:%q", backendCtx.Id, *h.Name, *h.Value)
		}
	}

	log.Printf("[%s] Wrote %d response bytes to request", backendCtx.Id, numBytes)
}

// relay-client pulls a request
func (s *Server) serverRequest(w http.ResponseWriter, r *http.Request) {
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

func (s *Server) serverRequestStream(w http.ResponseWriter, r *http.Request) {
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
func (s *Server) serverResponse(w http.ResponseWriter, r *http.Request) {
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

func (s *Server) Start(port int, blockSize int) {
	s.port = port
	s.blockSize = blockSize

	h := http.NewServeMux()
	h.HandleFunc("/healthz", s.health)
	h.HandleFunc("/", s.userClientRequest)
	h.HandleFunc("/server/request", s.serverRequest)
	h.HandleFunc("/server/requeststream", s.serverRequestStream)
	h.HandleFunc("/server/response", s.serverResponse)
	h.Handle("/metrics", promhttp.Handler())

	// This context will be terminated we get SIGTERM from Kubernetes. We need
	// some boilerplate to make this terminate the HTTP server and the ongoing
	// request contexts. This is based on:
	// https://www.rudderstack.com/blog/implementing-graceful-shutdown-in-go/#:~:text=Canceling%20long%20running%20requests
	mainCtx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	h2s := &http2.Server{}
	h2h := h2c.NewHandler(h, h2s)
	och := &ochttp.Handler{
		Handler: h2h,
	}
	h1s := &http.Server{
		Addr:    fmt.Sprintf(":%d", s.port),
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
