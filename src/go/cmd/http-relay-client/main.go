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
	"net/http"
	"net/url"
	pb "src/proto/http-relay"
	"sync"
	"time"

	"github.com/cenkalti/backoff"
	"github.com/golang/protobuf/proto"
	"golang.org/x/net/context"
	"golang.org/x/oauth2/google"
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
)

var (
	backendScheme = flag.String("backend_scheme", "https",
		"Connection scheme (http, https) for connection from relay "+
			"client to backend server")
	backendAddress = flag.String("backend_address", "localhost:8080",
		"Hostname of the backend server as seen by the relay client")
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
)

var (
	ErrTimeout = errors.New(http.StatusText(http.StatusRequestTimeout))
)

func getRequest(remote *http.Client) (*pb.HttpRequest, error) {
	log.Printf("Connecting to relay server to get next request for %s", *serverName)
	query := url.Values{}
	query.Add("server", *serverName)
	relayURL := url.URL{
		Scheme:   *relayScheme,
		Host:     *relayAddress,
		Path:     *relayPrefix + "/server/request",
		RawQuery: query.Encode(),
	}

	resp, err := remote.Get(relayURL.String())
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

func makeBackendRequest(local *http.Client, breq *pb.HttpRequest) (*pb.HttpResponse, io.ReadCloser, error) {
	id := *breq.Id
	targetUrl, err := url.Parse(*breq.Url)
	if err != nil {
		return nil, nil, err
	}
	targetUrl.Scheme = *backendScheme
	targetUrl.Host = *backendAddress
	log.Printf("Sending request to backend for %s: %s", id, targetUrl)
	req, err := http.NewRequest(*breq.Method, targetUrl.String(), bytes.NewReader(breq.Body))
	if err != nil {
		return nil, nil, err
	}
	for _, h := range breq.Header {
		req.Header.Set(*h.Name, *h.Value)
	}
	if *authenticationTokenFile != "" {
		token, err := ioutil.ReadFile(*authenticationTokenFile)
		if err != nil {
			return nil, nil, fmt.Errorf("Failed to read authentication token from %s: %v", *authenticationTokenFile, err)
		}
		req.Header.Set("Authorization", fmt.Sprintf("Bearer %s", token))
	}

	resp, err := local.Do(req)
	if err != nil {
		return nil, nil, err
	}

	log.Printf("Backend responded with %d to %s", resp.StatusCode, id)
	return &pb.HttpResponse{
		Id:         proto.String(id),
		StatusCode: proto.Int32(int32(resp.StatusCode)),
		Header:     marshalHeader(&resp.Header),
	}, resp.Body, nil
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
	resp, err := remote.Post(responseUrl.String(), "application/octet-data", bytes.NewReader(body))
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

	return nil
}

// streamBytes converts an io.Reader into a channel to enable select{}-style timeouts.
func streamBytes(in io.ReadCloser, out chan<- []byte) {
	eof := false
	for !eof {
		buffer := make([]byte, *blockSize)
		n, err := in.Read(buffer)
		if err != nil && err != io.EOF {
			log.Printf("Failed to read from http body stream: %v", err)
		}
		eof = err != nil
		if n > 0 {
			out <- buffer[:n]
		}
	}
	close(out)
	in.Close()
}

// buildResponses collates the bytes from the in stream into HttpResponse objects.
// This function needs to consider three cases:
//  - Data is coming fast. We chunk the data into 'maxChunkSize' blocks and keep sending it.
//  - Data is trickling slow. We accumulate data for the timeout duration and then send it.
//    Timeout is determined by the maximum latency the user should see.
//  - No data needs to be transferred. We keep sending empty responses every few seconds
//    to show the relay server that we're still alive.
func buildResponses(in <-chan []byte, resp *pb.HttpResponse, out chan<- *pb.HttpResponse, timeout time.Duration) {
	timer := time.NewTimer(timeout)
	timeouts := 0

ResponseLoop:
	for {
		select {
		case b, more := <-in:
			resp.Body = append(resp.Body, b...)
			if !more {
				log.Printf("Posting final response of %d bytes for %s", len(resp.Body), *resp.Id)
				resp.Eof = proto.Bool(true)
				out <- resp
				break ResponseLoop
			} else if len(resp.Body) > *maxChunkSize {
				log.Printf("Posting intermediate response of %d bytes for %s", len(resp.Body), *resp.Id)
				out <- resp
				resp = &pb.HttpResponse{Id: resp.Id}
				timeouts = 0
			}
		case <-timer.C:
			timer.Reset(timeout)
			timeouts += 1
			// We send an empty response after 30 timeouts as a keep-alive packet.
			if len(resp.Body) > 0 || resp.StatusCode != nil || timeouts > 30 {
				log.Printf("Posting partial response of %d bytes for %s", len(resp.Body), *resp.Id)
				out <- resp
				resp = &pb.HttpResponse{Id: resp.Id}
				timeouts = 0
			}
		}
	}
	close(out)
}

// postErrorResponse resolves the client's request in case of an internal error.
// This is not strictly necessary, but avoids kubectl hanging in such cases. As
// this is best-effort, errors posting the response are logged and ignored.
func postErrorResponse(remote *http.Client, req *pb.HttpRequest, message string) {
	resp := &pb.HttpResponse{
		Id:         req.Id,
		StatusCode: proto.Int32(http.StatusInternalServerError),
		Header: []*pb.HttpHeader{{
			Name:  proto.String("Content-Type"),
			Value: proto.String("text/plain"),
		}},
		Body: []byte(message),
		Eof:  proto.Bool(true),
	}
	if err := postResponse(remote, resp); err != nil {
		log.Printf("Failed to post error response for %s to relay: %v", *resp.Id, err)
	}
}

// streamToBackend streams data from the client (eg kubectl) to the
// backend. For example, when using `kubectl exec` this handles stdin.
// It fails permanently and closes the backend connection on any failure, as
// the relay-server doesn't have sufficiently advanced flow control to recover
// from dropped/duplicate "packets".
func streamToBackend(remote *http.Client, req *pb.HttpRequest, backendWriter io.WriteCloser) {
	// Close the backend connection on stream failure. This should cause the
	// response stream to end and prevent the client from hanging in the case
	// of an error in the request stream.
	defer backendWriter.Close()

	streamURL := (&url.URL{
		Scheme:   *relayScheme,
		Host:     *relayAddress,
		Path:     *relayPrefix + "/server/requeststream",
		RawQuery: "id=" + *req.Id,
	}).String()
	for {
		// Get data from the "request stream", then copy it to the backend.
		resp, err := remote.Post(streamURL, "text/plain", http.NoBody)
		if err != nil {
			// TODO(rodrigoq): detect transient failure and retry w/ backoff?
			// e.g. "server status Request Timeout: No request received within timeout"
			log.Printf("Failed to get request stream for %s: %v", *req.Id, err)
			return
		}
		defer resp.Body.Close()
		if resp.StatusCode == http.StatusGone {
			log.Printf("End of request stream for %s", *req.Id)
			return
		} else if resp.StatusCode != http.StatusOK {
			msg, err := ioutil.ReadAll(resp.Body)
			if err != nil {
				msg = []byte(fmt.Sprintf("<failed to read response body: %v>", err))
			}
			log.Printf("Relay server request stream responded %s: %s", http.StatusText(resp.StatusCode), msg)
			return
		}
		if n, err := io.Copy(backendWriter, resp.Body); err != nil {
			log.Printf("Failed to write to backend for %s: %v", *req.Id, err)
			return
		} else {
			log.Printf("Wrote %d bytes to request stream for %s", n, *req.Id)
		}
	}
}

func handleRequest(remote *http.Client, local *http.Client, req *pb.HttpRequest) {
	resp, body, err := makeBackendRequest(local, req)
	if err != nil {
		// Even if we couldn't handle the backend request, send an
		// answer to the relay that signals the error.
		log.Printf("Backend request failed, reporting this to the relay: %v", err)
		postErrorResponse(remote, req, fmt.Sprintf("Unable to reach backend: %v", err))
		return
	}

	if *resp.StatusCode == http.StatusSwitchingProtocols {
		// A 101 Switching Protocols response means that the request will be
		// used for bidirectional streaming, so start a goroutine to stream
		// from client to backend.
		bodyWriter, ok := body.(io.ReadWriteCloser)
		if !ok {
			log.Printf("Error: 101 Switching Protocols response with non-writable body.")
			log.Printf("       This occurs when using Go <1.12 or when http.Client.Timeout > 0.")
			postErrorResponse(remote, req, "Backend returned 101 Switching Protocols, which is not supported.")
			return
		}
		go streamToBackend(remote, req, bodyWriter)
	}

	bodyChannel := make(chan []byte)
	responseChannel := make(chan *pb.HttpResponse)
	go streamBytes(body, bodyChannel)
	go buildResponses(bodyChannel, resp, responseChannel, responseTimeout)

	exponentialBackoff := backoff.ExponentialBackOff{
		InitialInterval:     time.Second,
		RandomizationFactor: 0,
		Multiplier:          2,
		MaxInterval:         10 * time.Second,
		MaxElapsedTime:      0,
		Clock:               backoff.SystemClock,
	}

	for resp := range responseChannel {
		exponentialBackoff.Reset()
		err := backoff.RetryNotify(
			func() error {
				return postResponse(remote, resp)
			},
			backoff.WithMaxRetries(&exponentialBackoff, 10),
			func(err error, _ time.Duration) {
				log.Printf("Failed to post response for %s to relay: %v", *resp.Id, err)
			},
		)
		if _, ok := err.(*backoff.PermanentError); ok {
			// A permanent error suggests the request should be aborted.
			break
		}
	}
}

func localProxy(remote *http.Client, local *http.Client) error {
	req, err := getRequest(remote)
	if err != nil {
		if errors.Is(err, ErrTimeout) {
			return err
		} else {
			return fmt.Errorf("failed to get request from relay: %v", err)
		}
	}
	go handleRequest(remote, local, req)
	return nil
}

func localProxyWorker(remote *http.Client, local *http.Client) {
	for {
		err := localProxy(remote, local)
		if err != nil && !errors.Is(err, ErrTimeout) {
			log.Print(err)
			time.Sleep(1 * time.Second)
		}
	}
}

func main() {
	flag.Parse()

	ctx := context.Background()
	scope := "https://www.googleapis.com/auth/cloud-platform.read-only"
	remote, err := google.DefaultClient(ctx, scope)
	if err != nil {
		log.Fatalf("unable to set up credentials: %v", err)
	}
	remote.Timeout = remoteRequestTimeout
	transport := &http.Transport{}
	if *rootCAFile != "" {
		rootCAs := x509.NewCertPool()
		certs, err := ioutil.ReadFile(*rootCAFile)
		if err != nil {
			log.Fatalf("Failed to read CA file %s: %v", *rootCAFile, err)
		}
		if ok := rootCAs.AppendCertsFromPEM(certs); !ok {
			log.Fatalf("No certs found in %s", *rootCAFile)
		}
		transport.TLSClientConfig = &tls.Config{RootCAs: rootCAs}
	}

	// TODO(https://github.com/golang/go/issues/31391): reimplement timeouts if possible
	// (see also https://github.com/golang/go/issues/30876)
	local := &http.Client{Transport: transport}
	wg := new(sync.WaitGroup)
	wg.Add(*numPendingRequests)
	for i := 0; i < *numPendingRequests; i++ {
		go localProxyWorker(remote, local)
	}
	// Waiting for all goroutines to finish (they never do)
	wg.Wait()
}
