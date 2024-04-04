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

// Package main runs a local HTTP relay client.
//
// See the documentation of ../http-relay-server/main.go for details about
// the system architecture. In a nutshell, this program pulls serialized HTTP
// requests from a remote relay server, redirects them to a local backend, and
// posts the serialized response to the relay server.
package client

import (
	"bytes"
	"context"
	"crypto/tls"
	"crypto/x509"
	"errors"
	"fmt"
	"io"
	"log/slog"
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
	"github.com/googlecloudrobotics/ilog"

	"github.com/cenkalti/backoff"
	"go.opencensus.io/plugin/ochttp"
	"go.opencensus.io/plugin/ochttp/propagation/tracecontext"
	"go.opencensus.io/trace"
	"golang.org/x/net/http2"
	"golang.org/x/oauth2"
	"golang.org/x/oauth2/google"
	"google.golang.org/protobuf/proto"
)

var (
	ErrTimeout        = errors.New(http.StatusText(http.StatusRequestTimeout))
	ErrForbidden      = errors.New(http.StatusText(http.StatusForbidden))
	debugLogs    bool = false
)

// This is a package internal variable which we define to be able to overwrite
// the measured time during unit tests. This is a light weight alternative
// to mocking the entire time interface and passing it along all call paths.
var timeSince = time.Since

type ClientConfig struct {
	RemoteRequestTimeout   time.Duration
	BackendResponseTimeout time.Duration
	IdleConnTimeout        time.Duration
	ReadIdleTimeout        time.Duration

	DisableAuthForRemote    bool
	RootCAFile              string
	AuthenticationTokenFile string

	BackendScheme  string
	BackendAddress string
	BackendPath    string
	PreserveHost   bool

	RelayScheme  string
	RelayAddress string
	RelayPrefix  string

	ServerName string

	NumPendingRequests  int
	MaxIdleConnsPerHost int

	MaxChunkSize int
	BlockSize    int

	DisableHttp2 bool
	ForceHttp2   bool
}

type RelayServerError struct {
	msg string
}

func NewRelayServerError(msg string) error {
	return &RelayServerError{msg}
}

func (e *RelayServerError) Error() string {
	return e.msg
}

func DefaultClientConfig() ClientConfig {
	return ClientConfig{
		RemoteRequestTimeout:   60 * time.Second,
		BackendResponseTimeout: 100 * time.Millisecond,

		// ReadIdleTimeout works around an upstream issue by enabling
		// HTTP/2 PING, so we recover faster after the node IP changes.
		// IdleConnTimeout is here because I was worried this would
		// create unnecessary load with PINGs on long-idle connections.
		// https://github.com/golang/go/issues/59690
		ReadIdleTimeout: 30 * time.Second,
		IdleConnTimeout: 120 * time.Second,

		DisableAuthForRemote:    false,
		RootCAFile:              "",
		AuthenticationTokenFile: "",

		BackendScheme:  "https",
		BackendAddress: "localhost:8080",
		BackendPath:    "",
		PreserveHost:   true,

		RelayScheme:  "https",
		RelayAddress: "localhost:8081",
		RelayPrefix:  "",

		ServerName: "server_name",

		NumPendingRequests:  1,
		MaxIdleConnsPerHost: 100,

		MaxChunkSize: 50 * 1024,
		BlockSize:    10 * 1024,

		DisableHttp2: false,
		ForceHttp2:   false,
	}
}

type Client struct {
	config ClientConfig
}

func NewClient(config ClientConfig) *Client {
	c := &Client{}
	c.config = config
	return c
}

func (c *Client) Start() {
	var err error

	remoteTransport := http.DefaultTransport.(*http.Transport).Clone()
	remoteTransport.MaxIdleConns = c.config.MaxIdleConnsPerHost
	remoteTransport.MaxIdleConnsPerHost = c.config.MaxIdleConnsPerHost
	remoteTransport.IdleConnTimeout = c.config.IdleConnTimeout
	http2Trans, err := http2.ConfigureTransports(remoteTransport)
	if err == nil {
		http2Trans.ReadIdleTimeout = c.config.ReadIdleTimeout
	}
	remote := &http.Client{Transport: remoteTransport}

	if !c.config.DisableAuthForRemote {
		ctx := context.WithValue(context.Background(), oauth2.HTTPClient, remote)
		scope := "https://www.googleapis.com/auth/cloud-platform.read-only"
		if remote, err = google.DefaultClient(ctx, scope); err != nil {
			slog.Error("unable to set up credentials for relay-server authentication", ilog.Err(err))
			os.Exit(1)
		}
	}
	remote.Timeout = c.config.RemoteRequestTimeout

	var tlsConfig *tls.Config
	if c.config.RootCAFile != "" {
		rootCAs := x509.NewCertPool()
		certs, err := os.ReadFile(c.config.RootCAFile)
		if err != nil {
			slog.Error("Failed to read CA file", slog.String("File", c.config.RootCAFile), ilog.Err(err))
			os.Exit(1)
		}
		if ok := rootCAs.AppendCertsFromPEM(certs); !ok {
			slog.Error("No certs found", slog.String("File", c.config.RootCAFile))
			os.Exit(1)
		}
		tlsConfig = &tls.Config{RootCAs: rootCAs}

		if keyLogFile := os.Getenv("SSLKEYLOGFILE"); keyLogFile != "" {
			keyLog, err := os.OpenFile(keyLogFile, os.O_WRONLY|os.O_CREATE|os.O_TRUNC, 0600)
			if err != nil {
				slog.Warn("Cannot open keylog file (check SSLKEYLOGFILE env var)", slog.String("File", keyLogFile), ilog.Err(err))
			} else {
				tlsConfig.KeyLogWriter = keyLog
			}
		}
	}

	var transport http.RoundTripper
	if c.config.ForceHttp2 {
		h2transport := &http2.Transport{}
		h2transport.TLSClientConfig = tlsConfig

		if c.config.DisableHttp2 {
			slog.Error("Cannot use --force_http2 together with --disable_http2")
			os.Exit(1)
		}

		if c.config.BackendScheme == "http" {
			// Enable HTTP/2 Cleartext (H2C) for gRPC backends.
			h2transport.AllowHTTP = true
			h2transport.DialTLS = func(network, addr string, cfg *tls.Config) (net.Conn, error) {
				// Pretend we are dialing a TLS endpoint.
				// Note, we ignore the passed tls.Config
				return net.Dial(network, addr)
			}
		}

		transport = h2transport
	} else {
		h1transport := http.DefaultTransport.(*http.Transport).Clone()
		h1transport.MaxIdleConns = c.config.MaxIdleConnsPerHost
		h1transport.MaxIdleConnsPerHost = c.config.MaxIdleConnsPerHost
		h1transport.TLSClientConfig = tlsConfig

		if c.config.DisableHttp2 {
			// Fix for: http2: invalid Upgrade request header: ["SPDY/3.1"]
			// according to the docs:
			//    Programs that must disable HTTP/2 can do so by setting Transport.TLSNextProto (for clients) or
			//    Server.TLSNextProto (for servers) to a non-nil, empty map.
			//
			h1transport.TLSNextProto = map[string]func(authority string, c *tls.Conn) http.RoundTripper{}
		}

		transport = h1transport
	}

	// TODO(https://github.com/golang/go/issues/31391): reimplement timeouts if possible
	// (see also https://github.com/golang/go/issues/30876)
	local := &http.Client{
		CheckRedirect: func(*http.Request, []*http.Request) error {
			// Don't follow redirects: instead, pass them through the relay untouched.
			return http.ErrUseLastResponse
		},
		Transport: &ochttp.Transport{Base: transport},
	}

	wg := new(sync.WaitGroup)
	wg.Add(c.config.NumPendingRequests)
	for i := 0; i < c.config.NumPendingRequests; i++ {
		go c.localProxyWorker(remote, local)
	}
	// Waiting for all goroutines to finish (they never do)
	wg.Wait()
}

func addServiceName(span *trace.Span) {
	relayClientAttr := trace.StringAttribute("service.name", "http-relay-client")
	span.AddAttributes(relayClientAttr)
}

func (c *Client) getRequest(remote *http.Client, relayURL string) (*pb.HttpRequest, error) {
	if debugLogs {
		slog.Info("Connecting to relay server to get next request", slog.String("ServerName", c.config.ServerName))
	}

	resp, err := remote.Get(relayURL)
	if err != nil {
		return nil, err
	}
	defer resp.Body.Close()
	body, err := io.ReadAll(resp.Body)
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

func (c *Client) createBackendRequest(breq *pb.HttpRequest) (*http.Request, error) {
	id := *breq.Id
	targetUrl, err := url.Parse(*breq.Url)
	if err != nil {
		return nil, err
	}
	targetUrl.Scheme = c.config.BackendScheme
	targetUrl.Host = c.config.BackendAddress
	targetUrl.Path = c.config.BackendPath + targetUrl.Path
	slog.Debug("Sending request to backend",
		slog.String("ID", id),
		slog.String("Method", *breq.Method),
		slog.Any("TargetURL", *targetUrl))
	req, err := http.NewRequest(*breq.Method, targetUrl.String(), bytes.NewReader(breq.Body))
	if err != nil {
		return nil, err
	}
	if c.config.PreserveHost && breq.Host != nil {
		req.Host = *breq.Host
	}
	extractRequestHeader(breq, &req.Header)
	if c.config.AuthenticationTokenFile != "" {
		token, err := os.ReadFile(c.config.AuthenticationTokenFile)
		if err != nil {
			return nil, fmt.Errorf("Failed to read authentication token from %s: %v", c.config.AuthenticationTokenFile, err)
		}
		req.Header.Set("Authorization", fmt.Sprintf("Bearer %s", token))
	}

	if debugLogs {
		dump, _ := httputil.DumpRequest(req, false)
		slog.Info("DumpRequest", slog.String("Request", string(dump)))
	}

	return req, nil
}

// This function builds and executes a http.Request from the proto request we
// received from the user-client. This user-client (e.g. Chrome) request is
// executed in the network in which the relay-client is running. In case of
// our on-prem cluster, these requests are processed by Istio and sent to the
// relevant in-cluster service.
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
		slog.Info("Backend responded", slog.String("ID", id), slog.Int("Status", resp.StatusCode))

		dump, _ := httputil.DumpResponse(resp, false)
		slog.Info("DumpResponse", slog.String("Response", string(dump)))
		// We get 'Grpc-Status' and 'Grpc-Message' headers that we need to persist.
		// Why is it not part of Trailers?
		slog.Info("Headers",
			slog.String("ID", id),
			slog.String("Header", fmt.Sprintf("%+v", resp.Header)))
		// Initially only keys, values are set after body has be read (EOF)
		slog.Info("Trailers",
			slog.String("ID", id),
			slog.String("Trailer", fmt.Sprintf("%+v", resp.Trailer)))
	}

	return &pb.HttpResponse{
		Id:         proto.String(id),
		StatusCode: proto.Int32(int32(resp.StatusCode)),
		Header:     marshalHeader(&resp.Header),
		Trailer:    marshalHeader(&resp.Trailer),
	}, resp, nil
}

func (c *Client) postResponse(remote *http.Client, br *pb.HttpResponse) error {
	body, err := proto.Marshal(br)
	if err != nil {
		return err
	}

	responseUrl := url.URL{
		Scheme: c.config.RelayScheme,
		Host:   c.config.RelayAddress,
		Path:   c.config.RelayPrefix + "/server/response",
	}

	resp, err := remote.Post(responseUrl.String(), "application/vnd.google.protobuf;proto=cloudrobotics.http_relay.v1alpha1.HttpResponse", bytes.NewReader(body))

	if err != nil {
		return fmt.Errorf("couldn't post response to relay server: %v", err)
	}

	defer resp.Body.Close()
	body, err = io.ReadAll(resp.Body)
	if err != nil {
		return fmt.Errorf("couldn't read relay server's response body: %v", err)
	}
	if resp.StatusCode != http.StatusOK {
		err := NewRelayServerError(fmt.Sprintf("relay server responded %s: %s", http.StatusText(resp.StatusCode), body))
		if resp.StatusCode == http.StatusBadRequest {
			// http-relay-server may have restarted or the client cancelled the request.
			return backoff.Permanent(err)
		}
		return err
	}
	// body is only 2 bytes 'ok'
	return nil
}

// streamBytes converts an io.Reader into a channel to enable select{}-style timeouts.
func (c *Client) streamBytes(id string, in io.ReadCloser, out chan<- []byte) {
	eof := false
	for !eof {
		// This must be a new buffer each time, as the channel is not making a copy
		buffer := make([]byte, c.config.BlockSize)
		if debugLogs {
			slog.Info("Reading from backend", slog.String("ID", id))
		}
		n, err := in.Read(buffer)
		if err != nil && err != io.EOF {
			slog.Error("Failed to read from backend", slog.String("ID", id), ilog.Err(err))
		}
		eof = err != nil
		if n > 0 {
			if debugLogs {
				slog.Info("Forward from backend", slog.String("ID", id), slog.Int("ByteCount", n))
			}
			out <- buffer[:n]
		}
	}
	if debugLogs {
		slog.Info("Got EOF reading from backend", slog.String("ID", id))
	}
	close(out)
}

// buildResponses collates the bytes from the in stream into HttpResponse objects.
// This function needs to consider three cases:
//   - Data is coming fast. We chunk the data into 'maxChunkSize' blocks and keep sending it.
//   - Data is trickling slow. We accumulate data for the timeout duration and then send it.
//     Timeout is determined by the maximum latency the user should see.
//   - No data needs to be transferred. We keep sending empty responses every few seconds
//     to show the relay server that we're still alive.
func (c *Client) buildResponses(in <-chan []byte, resp *pb.HttpResponse, out chan<- *pb.HttpResponse) {
	defer close(out)
	timer := time.NewTimer(c.config.BackendResponseTimeout)
	timeouts := 0

	// TODO(haukeheibel): Why are we not simply reading the entire body? Why the chunking?
	for {
		select {
		case b, more := <-in:
			resp.Body = append(resp.Body, b...)
			if !more {
				if debugLogs {
					slog.Info("Posting final response to relay",
						slog.String("ID", *resp.Id), slog.Int("ByteCount", len(resp.Body)))
				}
				resp.Eof = proto.Bool(true)
				out <- resp
				return
			} else if len(resp.Body) > c.config.MaxChunkSize {
				if debugLogs {
					slog.Info("Posting intermediate response to relay",
						slog.String("ID", *resp.Id), slog.Int("ByteCount", len(resp.Body)))
				}
				out <- resp
				resp = &pb.HttpResponse{Id: resp.Id}
				timeouts = 0
			}
		case <-timer.C:
			timer.Reset(c.config.BackendResponseTimeout)
			timeouts += 1
			// We send an (empty) response after 30 timeouts as a keep-alive packet.
			if len(resp.Body) > 0 || resp.StatusCode != nil || timeouts > 30 {
				if debugLogs {
					slog.Info("Posting partial response to relay",
						slog.String("ID", *resp.Id), slog.Int("ByteCount", len(resp.Body)))
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
func (c *Client) postErrorResponse(remote *http.Client, id string, message string) {
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
	if err := c.postResponse(remote, resp); err != nil {
		slog.Error("Failed to post error response to relay",
			slog.String("ID", *resp.Id), ilog.Err(err))
	}
}

// streamToBackend streams data from the client (eg kubectl) to the
// backend. For example, when using `kubectl exec` this handles stdin.
// It fails permanently and closes the backend connection on any failure, as
// the relay-server doesn't have sufficiently advanced flow control to recover
// from dropped/duplicate "packets".
func (c *Client) streamToBackend(remote *http.Client, id string, backendWriter io.WriteCloser) {
	// Close the backend connection on stream failure. This should cause the
	// response stream to end and prevent the client from hanging in the case
	// of an error in the request stream.
	defer backendWriter.Close()

	streamURL := (&url.URL{
		Scheme:   c.config.RelayScheme,
		Host:     c.config.RelayAddress,
		Path:     c.config.RelayPrefix + "/server/requeststream",
		RawQuery: "id=" + id,
	}).String()
	for {
		// Get data from the "request stream", then copy it to the backend.
		// We use a Post with empty body to avoid caching.
		resp, err := remote.Post(streamURL, "text/plain", http.NoBody)
		if err != nil {
			// TODO(rodrigoq): detect transient failure and retry w/ backoff?
			// e.g. "server status Request Timeout: No request received within timeout"
			slog.Error("Failed to get request stream",
				slog.String("ID", id), ilog.Err(err))
			return
		}
		defer resp.Body.Close()
		if resp.StatusCode == http.StatusGone {
			if debugLogs {
				slog.Info("End of request stream", slog.String("ID", id))
			}
			return
		} else if resp.StatusCode != http.StatusOK {
			msg, err := io.ReadAll(resp.Body)
			if err != nil {
				msg = []byte(fmt.Sprintf("<failed to read response body: %v>", err))
			}
			if debugLogs {
				slog.Info("Relay server request stream responded",
					slog.String("ID", id),
					slog.String("Status", http.StatusText(resp.StatusCode)),
					slog.String("Message", string(msg)))

			}
			return
		}
		if n, err := io.Copy(backendWriter, resp.Body); err != nil {
			slog.Error("Failed to write to backend:",
				slog.String("ID", id), ilog.Err(err))
			return
		} else {
			if debugLogs {
				slog.Info("Wrote to backend",
					slog.String("ID", id), slog.Int64("ByteCount", n))
			}
		}
	}
}

func (c *Client) handleRequest(remote *http.Client, local *http.Client, pbreq *pb.HttpRequest) {
	ts := time.Now()
	id := *pbreq.Id
	req, err := c.createBackendRequest(pbreq)
	if err != nil {
		c.postErrorResponse(remote, id, fmt.Sprintf("Failed to create request for backend: %v", err))
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
		slog.Error("BackendRequest",
			slog.String("ID", id), slog.String("Message", errorMessage))
		c.postErrorResponse(remote, id, errorMessage)
		return
	}

	if *resp.StatusCode == http.StatusSwitchingProtocols {
		// A 101 Switching Protocols response means that the request will be
		// used for bidirectional streaming, so start a goroutine to stream
		// from client to backend.
		bodyWriter, ok := hresp.Body.(io.WriteCloser)
		if !ok {
			slog.Warn("Error: 101 Switching Protocols response with non-writable body.")
			slog.Warn("       This occurs when using Go <1.12 or when http.Client.Timeout > 0.")
			c.postErrorResponse(remote, id, "Backend returned 101 Switching Protocols, which is not supported.")
			return
		}
		// Stream stdin from remote to backend
		go c.streamToBackend(remote, id, bodyWriter)
	} else {
		// `streamToBackend` will close `hresp.Body` but it is only called on websocket connections.
		// We need to close it here for http connections.
		defer hresp.Body.Close()
	}

	ctx, respChSpan := trace.StartSpan(ctx, "Building (chunked) response channel")
	addServiceName(respChSpan)

	bodyChannel := make(chan []byte)
	responseChannel := make(chan *pb.HttpResponse)
	// Stream stdout from backend to bodyChannel
	go c.streamBytes(*resp.Id, hresp.Body, bodyChannel)
	// collect data from bodyChannel and send to remote (relay-server)
	go c.buildResponses(bodyChannel, resp, responseChannel)

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
					slog.Info("Trailers",
						slog.String("ID", *resp.Id),
						slog.String("Trailer", fmt.Sprintf("%+v", hresp.Trailer)))
					resp.Trailer = append(resp.Trailer, marshalHeader(&hresp.Trailer)...)
				}
				if resp.Eof != nil && *resp.Eof {
					duration := timeSince(ts)
					resp.BackendDurationMs = proto.Int64(duration.Milliseconds())
					// see makeBackendRequest()
					urlPath := strings.TrimPrefix(*pbreq.Url, "http://invalid")
					slog.Debug("Backend request",
						slog.String("ID", *resp.Id),
						slog.Float64("Duration", duration.Seconds()),
						slog.String("Path", urlPath))
				} else {
					// Q(hauke): When are we ending up in this branch?
					// What are the semantics and why are we not setting a request duration?
					// Even in a streaming case I would expect a duration which represents the
					// processing time of the last item.
				}
				return c.postResponse(remote, resp)
			},
			backoff.WithMaxRetries(&exponentialBackoff, 10),
			func(err error, _ time.Duration) {
				slog.Error("Failed to post response to relay",
					slog.String("ID", *resp.Id), ilog.Err(err))
			},
		)
		// Any error suggests the request should be aborted.
		// A missing chunk will cause clients to receive corrupted data, in most cases it is better
		// to close the connection to avoid that.
		if err != nil {
			slog.Error("Closing backend connection",
				slog.String("ID", *resp.Id), ilog.Err(err))
			break
		}
	}
}

func (c *Client) localProxy(remote, local *http.Client) error {
	// Read pending request from the relay-server.
	relayURL := c.buildRelayURL()

	var req *pb.HttpRequest = nil
	var err error = nil

	deadline := time.Now().Add(5 * time.Second)
	for time.Now().Before(deadline) {
		req, err = c.getRequest(remote, relayURL)
		if err != nil {
			if errors.Is(err, ErrTimeout) {
				return err
			} else if errors.Is(err, ErrForbidden) {
				slog.Error("failed to authenticate to cloud-api, restarting", ilog.Err(err))
				os.Exit(1)
			} else if errors.Is(err, syscall.ECONNREFUSED) {
				slog.Warn("Failed to connect to relay server. Retrying.")
				continue
			} else {
				return fmt.Errorf("failed to get request from relay: %v", err)
			}
		} else {
			break
		}
	}

	if err != nil {
		slog.Error("failed to connect to cloud-api, restarting", ilog.Err(err))
		os.Exit(1)
	}

	// Forward the request to the backend.
	go c.handleRequest(remote, local, req)
	return nil
}

func (c *Client) localProxyWorker(remote, local *http.Client) {
	slog.Info("Starting to relay server request loop", slog.String("ServerName", c.config.ServerName))
	for {
		err := c.localProxy(remote, local)
		if err != nil && !errors.Is(err, ErrTimeout) {
			slog.Error("localProxy", ilog.Err(err))
			time.Sleep(1 * time.Second)
		}
	}
}

func (c *Client) buildRelayURL() string {
	query := url.Values{}
	query.Add("server", c.config.ServerName)
	relayURL := url.URL{
		Scheme:   c.config.RelayScheme,
		Host:     c.config.RelayAddress,
		Path:     c.config.RelayPrefix + "/server/request",
		RawQuery: query.Encode(),
	}
	return relayURL.String()
}
