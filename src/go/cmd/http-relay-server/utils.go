// Utilities we use for implementing the relay server
package main

import (
	"crypto/rand"
	"encoding/hex"
	"fmt"
	"log"
	"net/http"
	"net/url"
	"strings"

	pb "github.com/googlecloudrobotics/core/src/proto/http-relay"
	"go.opencensus.io/trace"
	"google.golang.org/protobuf/proto"
)

const (
	clientPrefix = "/client/"
)

func addServiceName(span *trace.Span) {
	relayServerAttr := trace.StringAttribute("service.name", "http-relay-server")
	span.AddAttributes(relayServerAttr)
}

func createId() string {
	u := make([]byte, 16)
	// err is documented as always nil
	rand.Read(u)
	return hex.EncodeToString(u)
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

func createBackendRequest(backendCtx backendContext, r *http.Request, body []byte) *pb.HttpRequest {
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
		log.Printf("Writing header kv: %v = %v", *h.Name, *h.Value)
		w.Header().Add(*h.Name, *h.Value)
	}
}

func writeResponseHeader(backendCtx backendContext, w http.ResponseWriter, pres *pb.HttpResponse) {
	unmarshalHeader(w, pres.Header)

	// TODO(ensonic): open questions:
	// - can we do this less hacky? (see unmarshalHeader() above)
	// - why do we not always get them as trailers?
	for _, h := range pres.Header {
		if strings.HasPrefix(*h.Name, "Grpc-") {
			w.Header().Add(http.TrailerPrefix+*h.Name, *h.Value)
			log.Printf("[%s] Adding trailer from header: %q:%q", backendCtx.Id, *h.Name, *h.Value)
		}
	}
	for _, h := range pres.Trailer {
		w.Header().Add(http.TrailerPrefix+*h.Name, *h.Value)
		log.Printf("[%s] Adding real trailer: %q:%q", backendCtx.Id, *h.Name, *h.Value)
	}

	// It is important to call WriteHeader() only after w.Header().Add(...) has
	// been called for all parameters.
	w.WriteHeader(int(*pres.StatusCode))
}
