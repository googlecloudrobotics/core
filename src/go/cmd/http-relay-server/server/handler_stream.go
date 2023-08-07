package server

import (
	"log"
	"net/http"
	"strings"

	pb "github.com/googlecloudrobotics/core/src/proto/http-relay"
)

func bidirectionalStream(backendCtx backendContext, w http.ResponseWriter, backendResBodyChan <-chan []byte, streamToBackend chan []byte, blockSize int) {
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
			bytes := make([]byte, blockSize)
			// Here we get the user client stream (e.g. kubectl, k9s or browser stream)
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
			if streamToBackend != nil {
				streamToBackend <- bytes[:n]
			} else {
				log.Printf("[%s] End of bidi-stream stream", backendCtx.Id)
				return
			}
			log.Printf("[%s] Uploaded %d bytes from bidi-stream", backendCtx.Id, n)
		}
	}()

	numBytes := 0
	for backendResBody := range backendResBodyChan {
		// TODO(b/130706300): detect dropped connection and end request in broker
		_, _ = bufrw.Write(backendResBody)
		bufrw.Flush()
		numBytes += len(backendResBody)
	}
	log.Printf("[%s] Wrote %d response bytes to bidi-stream", backendCtx.Id, numBytes)
}

func createWebSocketBodyChannel(backendCtx backendContext, firstResponse *pb.HttpResponse, backendRespCh <-chan *pb.HttpResponse) <-chan []byte {
	websocketBodies := make(chan []byte, 1)
	websocketBodies <- []byte(firstResponse.Body)
	go func() {
		for backendResp := range backendRespCh {
			brokerResponses.WithLabelValues("client", "ok", backendCtx.ServerName, backendCtx.Path).Inc()
			websocketBodies <- []byte(backendResp.Body)
		}
		close(websocketBodies)
	}()
	return websocketBodies
}

func websocketRelayResponses(backendCtx backendContext, w http.ResponseWriter, firstBackendResp *pb.HttpResponse, backendRespCh <-chan *pb.HttpResponse, streamToBackend chan []byte, blockSize int) {
	writeResponseHeader(backendCtx, w, firstBackendResp)

	backendResBodyChan := createWebSocketBodyChannel(backendCtx, firstBackendResp, backendRespCh)
	bidirectionalStream(backendCtx, w, backendResBodyChan, streamToBackend, blockSize)

	// Even in the old code, we just returned here. Should we not handle the trailer of
	// the last response?
	// Old code: https://github.com/googlecloudrobotics/core/blob/238e8b52bc10b7a1f6b8a1e49c4a9fda423d2d07/src/go/cmd/http-relay-server/main.go#L334
}
