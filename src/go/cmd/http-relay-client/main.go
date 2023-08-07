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
	"flag"
	"log"
	"net/http"
	"time"

	"github.com/googlecloudrobotics/core/src/go/cmd/http-relay-client/client"

	"contrib.go.opencensus.io/exporter/stackdriver"
	"go.opencensus.io/trace"
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

func main() {
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

	config := client.ClientConfig{
		RemoteRequestTimeout:   60 * time.Second,
		BackendResponseTimeout: 100 * time.Millisecond,

		DisableAuthForRemote:    *disableAuthForRemote,
		RootCAFile:              *rootCAFile,
		AuthenticationTokenFile: *authenticationTokenFile,

		BackendScheme:  *backendScheme,
		BackendAddress: *backendAddress,
		BackendPath:    *backendPath,
		PreserveHost:   *preserveHost,

		RelayScheme:  *relayScheme,
		RelayAddress: *relayAddress,
		RelayPrefix:  *relayPrefix,

		ServerName: *serverName,

		NumPendingRequests:  *numPendingRequests,
		MaxIdleConnsPerHost: *maxIdleConnsPerHost,

		MaxChunkSize: *maxChunkSize,
		BlockSize:    *blockSize,

		DisableHttp2: *disableHttp2,
		ForceHttp2:   *forceHttp2,
	}
	client := client.NewClient(config)
	client.Start()
}
