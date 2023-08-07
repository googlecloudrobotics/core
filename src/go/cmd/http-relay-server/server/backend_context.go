package server

import (
	"crypto/rand"
	"encoding/hex"
	"fmt"
	"net/http"
	"strings"
)

const (
	clientPrefix = "/client/"
)

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
