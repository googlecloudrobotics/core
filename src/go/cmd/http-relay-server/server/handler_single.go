package server

import (
	"context"
	"log"
	"net/http"

	pb "github.com/googlecloudrobotics/core/src/proto/http-relay"
	"go.opencensus.io/trace"
)

func httpRelayResponses(ctx context.Context, backendCtx backendContext, w http.ResponseWriter, firstBackendResp *pb.HttpResponse, backendRespCh <-chan *pb.HttpResponse) {
	_, forwardingResponseSpan := trace.StartSpan(ctx, "Forwarding backend response to user-client")
	addServiceName(forwardingResponseSpan)
	defer forwardingResponseSpan.End()

	writeResponseHeader(backendCtx, w, firstBackendResp)

	numBytes := len(firstBackendResp.Body)
	w.Write(firstBackendResp.Body)
	log.Printf("[%s] Wrote %d response bytes to request", backendCtx.Id, numBytes)

	// We need to read more responses here because we do client-side chunking, even
	// in cases, where we are not working with websockets.
	for chunkResponse := range backendRespCh {
		// TODO(b/130706300): detect dropped connection and end request in broker
		_, _ = w.Write(chunkResponse.Body)
		if flush, ok := w.(http.Flusher); ok {
			flush.Flush()
		}
		numBytes += len(chunkResponse.Body)
		log.Printf("[%s] Wrote %d response bytes to request", backendCtx.Id, numBytes)
	}
}
