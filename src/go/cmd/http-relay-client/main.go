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

// Package main runs a local HTTP relay client.
//
// See the documentation of ../http-relay-server/main.go for details about
// the system architecture. In a nutshell, this program pulls serialized HTTP
// requests from a remote relay server, redirects them to a local backend, and
// posts the serialized response to the relay server.
package main

import (
	"bytes"
	"crypto/tls"
	"crypto/x509"
	"errors"
	"flag"
	"fmt"
	"io"
	"io/ioutil"
	"log"
	"net"
	"net/http"
	"net/http/httputil"
	"net/url"
	"os"
	"strings"
	"sync"
	"syscall"
	"time"

	pb "github.com/googlecloudrobotics/core/src/proto/http-relay"

	"contrib.go.opencensus.io/exporter/stackdriver"
	"github.com/cenkalti/backoff"
	"go.opencensus.io/plugin/ochttp"
	"go.opencensus.io/plugin/ochttp/propagation/tracecontext"
	"go.opencensus.io/trace"
	"golang.org/x/net/context"
	"golang.org/x/net/http2"
	"golang.org/x/oauth2/google"
	"google.golang.org/protobuf/proto"
)

const (
	// Timeout for requests to the relay-server. It should be longer than
	// the timeout in broker.GetRequest to avoid confusing error messages.
	// If it is too long, connection problems in getRequests could cause
	// the relay to stall indefinitely.
	remoteRequestTimeout = 60 * time.Second
	// When streaming response data to the client (eg `kubectl logs -f`), we
	// accumulate data to avoid sending too many requests to the relay-server.
	// responseTimeout specifies how long to accumulate before sending response
	// data. It is part of the round-trip latency when using `kubectl exec`.
	responseTimeout = 100 * time.Millisecond
	// Print more detailed logs when enabled.
	debugLogs = false
)

var (
	backendScheme = flag.String("backend_scheme", "https",
		"Connection scheme (http, https) for connection from relay "+
			"client to backend server")
	backendAddress = flag.String("backend_address", "localhost:8080",
		"Hostname of the backend server as seen by the relay client")
	backendPath = flag.String("backend_path", "",
		"Path prefix for backend requests (default: none)")
	preserveHost = flag.Bool("preserve_host", true,
		"Preserve Host header of the original request for "+
			"compatibility with cross-origin request checks.")
	relayScheme = flag.String("relay_scheme", "https",
		"Connection scheme (http, https) for connection from relay "+
			"client to relay server")
	relayAddress = flag.String("relay_address", "localhost:8081",
		"Hostname of the relay server as seen by the relay client")
	relayPrefix = flag.String("relay_prefix", "",
		"Path prefix for the relay server")
	serverName = flag.String("server_name", "foo", "Fetch requests from "+
		"the relay server for this server name")
	authenticationTokenFile = flag.String("authentication_token_file", "",
		"File with authentication token for backend requests")
	rootCAFile = flag.String("root_ca_file", "",
		"File with root CA cert for SSL")
	maxChunkSize = flag.Int("max_chunk_size", 50*1024,
		"Max size of data in bytes to accumulate before sending to the peer")
	blockSize = flag.Int("block_size", 10*1024,
		"Size of i/o buffer in bytes")
	numPendingRequests = flag.Int("num_pending_requests", 1,
		"Number of pending http requests to the relay")
	maxIdleConnsPerHost = flag.Int("max_idle_conns_per_host", http.DefaultMaxIdleConnsPerHost,
		"The maximum number of idle (keep-alive) connections to keep per-host")
	disableHttp2 = flag.Bool("disable_http2", false,
		"Disable http2 protocol usage (e.g. for channels that use special streaming protocols such as SPDY).")
	forceHttp2 = flag.Bool("force_http2", false,
		"Force enable http2 protocol usage through the use of go's http2 transport (e.g. when relaying grpc).")
	disableAuthForRemote = flag.Bool("disable_auth_for_remote", false,
		"Disable auth when talking to the relay server for local testing.")
	stackdriverProjectID = flag.String("trace-stackdriver-project-id", "",
		"If not empty, traces will be uploaded to this Google Cloud Project.")
)

var (
	ErrTimeout   = errors.New(http.StatusText(http.StatusRequestTimeout))
	ErrForbidden = errors.New(http.StatusText(http.StatusForbidden))
)

func addServiceName(span *trace.Span) {
	relayClientAttr := trace.StringAttribute("service.name", "http-relay-client")
	span.AddAttributes(relayClientAttr)
}

func getRequest(remote *http.Client, relayURL string) (*pb.HttpRequest, error) {
	if debugLogs {
		log.Printf("Connecting to relay server to get next request for %s", *serverName)
	}

	resp, err := remote.Get(relayURL)
	if err != nil {
		return nil, err
	}
	defer resp.Body.Close()
	body, err := ioutil.ReadAll(resp.Body)
	if err != nil {
		return nil, err
	}

	if resp.StatusCode == http.StatusRequestTimeout {
		return nil, ErrTimeout
	}
	if resp.StatusCode == http.StatusForbidden {
		return nil, ErrForbidden
	}
	if resp.StatusCode != http.StatusOK {
		return nil, fmt.Errorf("server status %s: %s", http.StatusText(resp.StatusCode), string(body))
	}
	breq := pb.HttpRequest{}
	err = proto.Unmarshal(body, &breq)
	if err != nil {
		return nil, fmt.Errorf("failed to unmarshal request: %v. request was: %q", err, string(body))
	}

	return &breq, nil
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

func extractRequestHeader(breq *pb.HttpRequest, header *http.Header) {
	for _, h := range breq.Header {
		header.Add(*h.Name, *h.Value)
	}
}

func createBackendRequest(breq *pb.HttpRequest) (*http.Request, error) {
	id := *breq.Id
	targetUrl, err := url.Parse(*breq.Url)
	targetUrl.Scheme = *backendScheme
	targetUrl.Host = *backendAddress
	targetUrl.Path = *backendPath + targetUrl.Path
	log.Printf("[%s] Sending %s request to backend: %s", id, *breq.Method, targetUrl)
	req, err := http.NewRequest(*breq.Method, targetUrl.String(), bytes.NewReader(breq.Body))
	if err != nil {
		return nil, err
	}
	if *preserveHost && breq.Host != nil {
		req.Host = *breq.Host
	}
	extractRequestHeader(breq, &req.Header)
	if *authenticationTokenFile != "" {
		token, err := ioutil.ReadFile(*authenticationTokenFile)
		if err != nil {
			return nil, fmt.Errorf("Failed to read authentication token from %s: %v", *authenticationTokenFile, err)
		}
		req.Header.Set("Authorization", fmt.Sprintf("Bearer %s", token))
	}

	if debugLogs {
		dump, _ := httputil.DumpRequest(req, false)
		log.Printf("%s", dump)
	}

	return req, nil
}

// This function builds and executes a http.Request from the proto request we received from the web-client.
// This web-client (e.g. Chrome) request is executed in the network in which the relay-client is running.
// In case of our on-prem cluster, these requests are processed by Istio and sent to the relevant in-cluster

// It returns both a new pb.HttpResponse as well as the related http.Response so
// that the caller can access e.g. http trailers once the response body has
// been read.
func makeBackendRequest(ctx context.Context, local *http.Client, req *http.Request, id string) (*pb.HttpResponse, *http.Response, error) {
	_, backendSpan := trace.StartSpan(ctx, "Sent."+req.URL.Path)
	addServiceName(backendSpan)
	f := &tracecontext.HTTPFormat{}
	f.SpanContextToRequest(backendSpan.SpanContext(), req)
	resp, err := local.Do(req)
	if err != nil {
		backendSpan.End()
		return nil, nil, err
	}
	backendSpan.End()

	_, backendResp := trace.StartSpan(ctx, "Creating response (proto marshaling)")
	addServiceName(backendResp)
	defer backendResp.End()

	if debugLogs {
		log.Printf("[%s] Backend responded with status %d", id, resp.StatusCode)

		dump, _ := httputil.DumpResponse(resp, false)
		log.Printf("%s", dump)
		// We get 'Grpc-Status' and 'Grpc-Message' headers that we need to persist.
		// Why is it not part of Trailers?
		log.Printf("[%s] Headers: %+v", id, resp.Header)
		// Initially only keys, values are set after body has be read (EOF)
		log.Printf("[%s] Trailers: %+v", id, resp.Trailer)
	}

	return &pb.HttpResponse{
		Id:         proto.String(id),
		StatusCode: proto.Int32(int32(resp.StatusCode)),
		Header:     marshalHeader(&resp.Header),
		Trailer:    marshalHeader(&resp.Trailer),
	}, resp, nil
}

func postResponse(remote *http.Client, br *pb.HttpResponse) error {
	body, err := proto.Marshal(br)
	if err != nil {
		return err
	}

	responseUrl := url.URL{
		Scheme: *relayScheme,
		Host:   *relayAddress,
		Path:   *relayPrefix + "/server/response",
	}

	resp, err := remote.Post(responseUrl.String(), "application/vnd.google.protobuf;proto=cloudrobotics.http_relay.v1alpha1.HttpResponse", bytes.NewReader(body))

	if err != nil {
		return fmt.Errorf("couldn't post response to relay server: %v", err)
	}

	defer resp.Body.Close()
	body, err = ioutil.ReadAll(resp.Body)
	if err != nil {
		return fmt.Errorf("couldn't read relay server's response body: %v", err)
	}
	if resp.StatusCode != http.StatusOK {
		err := fmt.Errorf("relay server responded %s: %s", http.StatusText(resp.StatusCode), body)
		if resp.StatusCode == http.StatusBadRequest {
			// http-relay-server may have restarted during the request.
			return backoff.Permanent(err)
		}
		return err
	}
	// body is only 2 bytes 'ok'
	return nil
}

// streamBytes converts an io.Reader into a channel to enable select{}-style timeouts.
func streamBytes(id string, in io.ReadCloser, out chan<- []byte) {
	eof := false
	for !eof {
		// This must be a new buffer each time, as the channel is not making a copy
		buffer := make([]byte, *blockSize)
		if debugLogs {
			log.Printf("[%s] Reading from backend", id)
		}
		n, err := in.Read(buffer)
		if err != nil && err != io.EOF {
			log.Printf("[%s] Failed to read from backend: %v", id, err)
		}
		eof = err != nil
		if n > 0 {
			if debugLogs {
				log.Printf("[%s] Forward %d bytes from backend", id, n)
			}
			out <- buffer[:n]
		}
	}
	if debugLogs {
		log.Printf("[%s] Got EOF reading from backend", id)
	}
	close(out)
	in.Close()
}

// buildResponses collates the bytes from the in stream into HttpResponse objects.
// This function needs to consider three cases:
//   - Data is coming fast. We chunk the data into 'maxChunkSize' blocks and keep sending it.
//   - Data is trickling slow. We accumulate data for the timeout duration and then send it.
//     Timeout is determined by the maximum latency the user should see.
//   - No data needs to be transferred. We keep sending empty responses every few seconds
//     to show the relay server that we're still alive.
func buildResponses(in <-chan []byte, resp *pb.HttpResponse, out chan<- *pb.HttpResponse, timeout time.Duration) {
	defer close(out)
	timer := time.NewTimer(timeout)
	timeouts := 0

	// TODO(haukeheibel): Why are we not simply reading the entire body? Why the chunking?
	for {
		select {
		case b, more := <-in:
			resp.Body = append(resp.Body, b...)
			if !more {
				if debugLogs {
					log.Printf("[%s] Posting final response of %d bytes to relay", *resp.Id, len(resp.Body))
				}
				resp.Eof = proto.Bool(true)
				out <- resp
				return
			} else if len(resp.Body) > *maxChunkSize {
				if debugLogs {
					log.Printf("[%s] Posting intermediate response of %d bytes to relay", *resp.Id, len(resp.Body))
				}
				out <- resp
				resp = &pb.HttpResponse{Id: resp.Id}
				timeouts = 0
			}
		case <-timer.C:
			timer.Reset(timeout)
			timeouts += 1
			// We send an (empty) response after 30 timeouts as a keep-alive packet.
			if len(resp.Body) > 0 || resp.StatusCode != nil || timeouts > 30 {
				if debugLogs {
					log.Printf("[%s] Posting partial response of %d bytes to relay", *resp.Id, len(resp.Body))
				}
				out <- resp
				resp = &pb.HttpResponse{Id: resp.Id}
				timeouts = 0
			}
		}
	}
}

// postErrorResponse resolves the client's request in case of an internal error.
// This is not strictly necessary, but avoids kubectl hanging in such cases. As
// this is best-effort, errors posting the response are logged and ignored.
func postErrorResponse(remote *http.Client, id string, message string) {
	resp := &pb.HttpResponse{
		Id:         proto.String(id),
		StatusCode: proto.Int32(http.StatusInternalServerError),
		Header: []*pb.HttpHeader{{
			Name:  proto.String("Content-Type"),
			Value: proto.String("text/plain"),
		}},
		Body: []byte(message),
		Eof:  proto.Bool(true),
	}
	if err := postResponse(remote, resp); err != nil {
		log.Printf("[%s] Failed to post error response to relay: %v", *resp.Id, err)
	}
}

// streamToBackend streams data from the client (eg kubectl) to the
// backend. For example, when using `kubectl exec` this handles stdin.
// It fails permanently and closes the backend connection on any failure, as
// the relay-server doesn't have sufficiently advanced flow control to recover
// from dropped/duplicate "packets".
func streamToBackend(remote *http.Client, id string, backendWriter io.WriteCloser) {
	// Close the backend connection on stream failure. This should cause the
	// response stream to end and prevent the client from hanging in the case
	// of an error in the request stream.
	defer backendWriter.Close()

	streamURL := (&url.URL{
		Scheme:   *relayScheme,
		Host:     *relayAddress,
		Path:     *relayPrefix + "/server/requeststream",
		RawQuery: "id=" + id,
	}).String()
	for {
		// Get data from the "request stream", then copy it to the backend.
		// We use a Post with empty body to avoid caching.
		resp, err := remote.Post(streamURL, "text/plain", http.NoBody)
		if err != nil {
			// TODO(rodrigoq): detect transient failure and retry w/ backoff?
			// e.g. "server status Request Timeout: No request received within timeout"
			log.Printf("[%s] Failed to get request stream: %v", id, err)
			return
		}
		defer resp.Body.Close()
		if resp.StatusCode == http.StatusGone {
			if debugLogs {
				log.Printf("[%s] End of request stream", id)
			}
			return
		} else if resp.StatusCode != http.StatusOK {
			msg, err := ioutil.ReadAll(resp.Body)
			if err != nil {
				msg = []byte(fmt.Sprintf("<failed to read response body: %v>", err))
			}
			if debugLogs {
				log.Printf("[%s] Relay server request stream responded %s: %s", id, http.StatusText(resp.StatusCode), msg)
			}
			return
		}
		if n, err := io.Copy(backendWriter, resp.Body); err != nil {
			log.Printf("[%s] Failed to write to backend: %v", id, err)
			return
		} else {
			if debugLogs {
				log.Printf("[%s] Wrote %d bytes to backend", id, n)
			}
		}
	}
}

func handleRequest(remote *http.Client, local *http.Client, pbreq *pb.HttpRequest) {
	ts := time.Now()
	id := *pbreq.Id
	req, err := createBackendRequest(pbreq)
	if err != nil {
		postErrorResponse(remote, id, fmt.Sprintf("Failed to create request for backend: %v", err))
	}
	// Measure edge processing time.
	f := &tracecontext.HTTPFormat{}
	ctx := req.Context()
	var span *trace.Span
	if sctx, ok := f.SpanContextFromRequest(req); ok {
		ctx, span = trace.StartSpanWithRemoteParent(ctx, "Recv."+req.URL.Path, sctx)
	} else {
		ctx, span = trace.StartSpan(ctx, "Recv."+req.URL.Path)
	}
	addServiceName(span)
	defer span.End()

	resp, hresp, err := makeBackendRequest(ctx, local, req, id)
	if err != nil {
		// Even if we couldn't handle the backend request, send an
		// answer to the relay that signals the error.
		errorMessage := fmt.Sprintf("Backend request failed with error: %v", err)
		log.Printf("[%s] %s", id, errorMessage)
		postErrorResponse(remote, id, errorMessage)
		return
	}
	// hresp.Body is either closed from streamToBackend() or streamBytes()

	if *resp.StatusCode == http.StatusSwitchingProtocols {
		// A 101 Switching Protocols response means that the request will be
		// used for bidirectional streaming, so start a goroutine to stream
		// from client to backend.
		bodyWriter, ok := hresp.Body.(io.WriteCloser)
		if !ok {
			log.Printf("Error: 101 Switching Protocols response with non-writable body.")
			log.Printf("       This occurs when using Go <1.12 or when http.Client.Timeout > 0.")
			postErrorResponse(remote, id, "Backend returned 101 Switching Protocols, which is not supported.")
			return
		}
		// Stream stdin from remote to backend
		go streamToBackend(remote, id, bodyWriter)
	}

	ctx, respChSpan := trace.StartSpan(ctx, "Building (chunked) response channel")
	addServiceName(respChSpan)

	bodyChannel := make(chan []byte)
	responseChannel := make(chan *pb.HttpResponse)
	// Stream stdout from backend to bodyChannel
	go streamBytes(*resp.Id, hresp.Body, bodyChannel)
	// collect data from bodyChannel and send to remote (relay-server)
	go buildResponses(bodyChannel, resp, responseChannel, responseTimeout)

	respChSpan.End()

	exponentialBackoff := backoff.ExponentialBackOff{
		InitialInterval:     time.Second,
		RandomizationFactor: 0,
		Multiplier:          2,
		MaxInterval:         10 * time.Second,
		MaxElapsedTime:      0,
		Clock:               backoff.SystemClock,
	}

	// This call here blocks until all data from the bodyChannel has been read.
	for resp := range responseChannel {
		_, respCh := trace.StartSpan(ctx, "Sending response from channel")
		addServiceName(respCh)
		defer respCh.End()

		// Q(hauke): do we really need exponential backoff in the relay?
		exponentialBackoff.Reset()
		err := backoff.RetryNotify(
			func() error {
				if len(hresp.Trailer) > 0 {
					log.Printf("[%s] Trailers: %+v", *resp.Id, hresp.Trailer)
					resp.Trailer = append(resp.Trailer, marshalHeader(&hresp.Trailer)...)
				}
				if resp.Eof != nil && *resp.Eof {
					duration := time.Since(ts)
					resp.BackendDurationMs = proto.Int64(duration.Milliseconds())
					// see makeBackendRequest()
					urlPath := strings.TrimPrefix(*pbreq.Url, "http://invalid")
					log.Printf("[%s] Backend request duration: %.3fs (for %s)", *resp.Id, duration.Seconds(), urlPath)
				} else {
					// Q(hauke): When are we ending up in this branch?
					// What are the semantics and why are we not setting a request duration?
					// Even in a streaming case I would expect a duration which represents the
					// processing time of the last item.
				}
				err = postResponse(remote, resp)
				return err
			},
			backoff.WithMaxRetries(&exponentialBackoff, 10),
			func(err error, _ time.Duration) {
				log.Printf("[%s] Failed to post response to relay: %v", *resp.Id, err)
			},
		)
		if _, ok := err.(*backoff.PermanentError); ok {
			// A permanent error suggests the request should be aborted.
			break
		}
	}
}

func localProxy(remote, local *http.Client, relayURL string) error {
	// Read pending request from the relay-server.
	req, err := getRequest(remote, relayURL)
	if err != nil {
		if errors.Is(err, ErrTimeout) {
			return err
		} else if errors.Is(err, ErrForbidden) {
			log.Fatalf("failed to authenticate to cloud-api, restarting: %v", err)
		} else if errors.Is(err, syscall.ECONNREFUSED) {
			log.Fatalf("failed to connect to cloud-api, restarting: %v", err)
		} else {
			return fmt.Errorf("failed to get request from relay: %v", err)
		}
	}
	// Forward the request to the backend.
	go handleRequest(remote, local, req)
	return nil
}

func localProxyWorker(remote, local *http.Client, relayURL string) {
	log.Printf("Starting to relay server request loop for %s", *serverName)
	for {
		err := localProxy(remote, local, relayURL)
		if err != nil && !errors.Is(err, ErrTimeout) {
			log.Print(err)
			time.Sleep(1 * time.Second)
		}
	}
}

func buildRelayURL() string {
	query := url.Values{}
	query.Add("server", *serverName)
	relayURL := url.URL{
		Scheme:   *relayScheme,
		Host:     *relayAddress,
		Path:     *relayPrefix + "/server/request",
		RawQuery: query.Encode(),
	}
	return relayURL.String()
}

func main() {
	var err error

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

	remote := &http.Client{}
	if !*disableAuthForRemote {
		ctx := context.Background()
		scope := "https://www.googleapis.com/auth/cloud-platform.read-only"
		if remote, err = google.DefaultClient(ctx, scope); err != nil {
			log.Fatalf("unable to set up credentials for relay-server authentication: %v", err)
		}
	}
	remote.Timeout = remoteRequestTimeout

	var tlsConfig *tls.Config
	if *rootCAFile != "" {
		rootCAs := x509.NewCertPool()
		certs, err := ioutil.ReadFile(*rootCAFile)
		if err != nil {
			log.Fatalf("Failed to read CA file %s: %v", *rootCAFile, err)
		}
		if ok := rootCAs.AppendCertsFromPEM(certs); !ok {
			log.Fatalf("No certs found in %s", *rootCAFile)
		}
		tlsConfig = &tls.Config{RootCAs: rootCAs}

		if keyLogFile := os.Getenv("SSLKEYLOGFILE"); keyLogFile != "" {
			keyLog, err := os.OpenFile(keyLogFile, os.O_WRONLY|os.O_CREATE|os.O_TRUNC, 0600)
			if err != nil {
				log.Printf("Can open keylog file %q (check SSLKEYLOGFILE env var): %v", keyLogFile, err)
			} else {
				tlsConfig.KeyLogWriter = keyLog
			}
		}
	}

	var transport http.RoundTripper
	if *forceHttp2 {
		h2transport := &http2.Transport{}
		h2transport.TLSClientConfig = tlsConfig

		if *disableHttp2 {
			log.Fatal("Cannot use --force_http2 together with --disable_http2")
		}

		if *backendScheme == "http" {
			// Enable HTTP/2 Cleartext (H2C) for gRPC backends.
			h2transport.AllowHTTP = true
			h2transport.DialTLS = func(network, addr string, cfg *tls.Config) (net.Conn, error) {
				// Pretend we are dialing a TLS endpoint.
				// Note, we ignore the passed tls.Config
				return net.Dial(network, addr)
			}
		}

		transport = &ochttp.Transport{Base: h2transport}
	} else {
		h1transport := http.DefaultTransport.(*http.Transport).Clone()
		h1transport.MaxIdleConnsPerHost = *maxIdleConnsPerHost
		h1transport.TLSClientConfig = tlsConfig

		if *disableHttp2 {
			// Fix for: http2: invalid Upgrade request header: ["SPDY/3.1"]
			// according to the docs:
			//    Programs that must disable HTTP/2 can do so by setting Transport.TLSNextProto (for clients) or
			//    Server.TLSNextProto (for servers) to a non-nil, empty map.
			//
			h1transport.TLSNextProto = map[string]func(authority string, c *tls.Conn) http.RoundTripper{}
		}

		transport = &ochttp.Transport{Base: h1transport}
	}

	// TODO(https://github.com/golang/go/issues/31391): reimplement timeouts if possible
	// (see also https://github.com/golang/go/issues/30876)
	local := &http.Client{
		CheckRedirect: func(*http.Request, []*http.Request) error {
			// Don't follow redirects: instead, pass them through the relay untouched.
			return http.ErrUseLastResponse
		},
		Transport: transport,
	}

	relayURL := buildRelayURL()

	wg := new(sync.WaitGroup)
	wg.Add(*numPendingRequests)
	for i := 0; i < *numPendingRequests; i++ {
		go localProxyWorker(remote, local, relayURL)
	}
	// Waiting for all goroutines to finish (they never do)
	wg.Wait()
}
