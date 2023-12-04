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
package main

import (
	"flag"
	"log/slog"
	"os"
	"time"

	"contrib.go.opencensus.io/exporter/stackdriver"
	"github.com/googlecloudrobotics/core/src/go/cmd/http-relay-client/client"
	"github.com/googlecloudrobotics/ilog"
	"go.opencensus.io/trace"
)

var (
	backendScheme           string
	backendAddress          string
	backendPath             string
	preserveHost            bool
	relayScheme             string
	relayAddress            string
	relayPrefix             string
	serverName              string
	authenticationTokenFile string
	rootCAFile              string
	maxChunkSize            int
	blockSize               int
	numPendingRequests      int
	maxIdleConnsPerHost     int
	disableHttp2            bool
	forceHttp2              bool
	disableAuthForRemote    bool
	stackdriverProjectID    string
)

func init() {
	dcc := client.DefaultClientConfig()

	// We set the default values for all command line flags to be equal to the
	// values in the default client config to ensure consistency between the two.
	flag.StringVar(&backendScheme, "backend_scheme", dcc.BackendScheme,
		"Connection scheme (http, https) for connection from relay "+
			"client to backend server")
	flag.StringVar(&backendAddress, "backend_address", dcc.BackendAddress,
		"Hostname of the backend server as seen by the relay client")
	flag.StringVar(&backendPath, "backend_path", dcc.BackendPath,
		"Path prefix for backend requests (default: none)")
	flag.BoolVar(&preserveHost, "preserve_host", dcc.PreserveHost,
		"Preserve Host header of the original request for "+
			"compatibility with cross-origin request checks.")
	flag.StringVar(&relayScheme, "relay_scheme", dcc.RelayScheme,
		"Connection scheme (http, https) for connection from relay "+
			"client to relay server")
	flag.StringVar(&relayAddress, "relay_address", dcc.RelayAddress,
		"Hostname of the relay server as seen by the relay client")
	flag.StringVar(&relayPrefix, "relay_prefix", dcc.RelayPrefix,
		"Path prefix for the relay server")
	flag.StringVar(&serverName, "server_name", dcc.ServerName, "Fetch requests from "+
		"the relay server for this server name")
	flag.StringVar(&authenticationTokenFile, "authentication_token_file", dcc.AuthenticationTokenFile,
		"File with authentication token for backend requests")
	flag.StringVar(&rootCAFile, "root_ca_file", dcc.RootCAFile,
		"File with root CA cert for SSL")
	flag.IntVar(&maxChunkSize, "max_chunk_size", dcc.MaxChunkSize,
		"Max size of data in bytes to accumulate before sending to the peer")
	flag.IntVar(&blockSize, "block_size", dcc.BlockSize,
		"Size of i/o buffer in bytes")
	flag.IntVar(&numPendingRequests, "num_pending_requests", dcc.NumPendingRequests,
		"Number of pending http requests to the relay")
	flag.IntVar(&maxIdleConnsPerHost, "max_idle_conns_per_host", dcc.MaxIdleConnsPerHost,
		"The maximum number of idle (keep-alive) connections to keep per-host")
	flag.BoolVar(&disableHttp2, "disable_http2", dcc.DisableHttp2,
		"Disable http2 protocol usage (e.g. for channels that use special streaming protocols such as SPDY).")
	flag.BoolVar(&forceHttp2, "force_http2", dcc.ForceHttp2,
		"Force enable http2 protocol usage through the use of go's http2 transport (e.g. when relaying grpc).")
	flag.BoolVar(&disableAuthForRemote, "disable_auth_for_remote", dcc.DisableAuthForRemote,
		"Disable auth when talking to the relay server for local testing.")

	// The stackdriver project ID is a client independent variable and so we
	// initialize it independently.
	flag.StringVar(&stackdriverProjectID, "trace-stackdriver-project-id", "",
		"If not empty, traces will be uploaded to this Google Cloud Project.")
}

func main() {
	flag.Parse()

	logHandler := ilog.NewLogHandler(slog.LevelInfo, os.Stdout)
	slog.SetDefault(slog.New(logHandler))

	if stackdriverProjectID != "" {
		sd, err := stackdriver.NewExporter(stackdriver.Options{
			ProjectID: stackdriverProjectID,
		})
		if err != nil {
			slog.Error("Failed to create the Stackdriver exporter", slog.String("Project", stackdriverProjectID), ilog.Err(err))
			os.Exit(1)
		} else {
			trace.RegisterExporter(sd)
			defer sd.Flush()
		}
	}

	config := client.ClientConfig{
		RemoteRequestTimeout:   60 * time.Second,
		BackendResponseTimeout: 100 * time.Millisecond,

		DisableAuthForRemote:    disableAuthForRemote,
		RootCAFile:              rootCAFile,
		AuthenticationTokenFile: authenticationTokenFile,

		BackendScheme:  backendScheme,
		BackendAddress: backendAddress,
		BackendPath:    backendPath,
		PreserveHost:   preserveHost,

		RelayScheme:  relayScheme,
		RelayAddress: relayAddress,
		RelayPrefix:  relayPrefix,

		ServerName: serverName,

		NumPendingRequests:  numPendingRequests,
		MaxIdleConnsPerHost: maxIdleConnsPerHost,

		MaxChunkSize: maxChunkSize,
		BlockSize:    blockSize,

		DisableHttp2: disableHttp2,
		ForceHttp2:   forceHttp2,
	}
	client := client.NewClient(config)
	client.Start()
}
