// Utilities we use for implementing the relay server
package server

import (
	"log"
	"net/http"
	"net/url"
	"strings"

	pb "github.com/googlecloudrobotics/core/src/proto/http-relay"
	"go.opencensus.io/trace"
	"google.golang.org/protobuf/proto"
)

func addServiceName(span *trace.Span) {
	relayServerAttr := trace.StringAttribute("service.name", "http-relay-server")
	span.AddAttributes(relayServerAttr)
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
