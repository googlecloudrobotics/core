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

	"contrib.go.opencensus.io/exporter/stackdriver"
	"github.com/googlecloudrobotics/core/src/go/cmd/http-relay-client/client"
	"github.com/googlecloudrobotics/ilog"
	"go.opencensus.io/trace"
)

var (
	config client.ClientConfig

	stackdriverProjectID string
)

func init() {
	config = client.DefaultClientConfig()

	// We set the default values for all command line flags to be equal to the
	// values in the default client config to ensure consistency between the two.
	flag.StringVar(&config.BackendScheme, "backend_scheme", config.BackendScheme,
		"Connection scheme (http, https) for connection from relay "+
			"client to backend server")
	flag.StringVar(&config.BackendAddress, "backend_address", config.BackendAddress,
		"Hostname of the backend server as seen by the relay client")
	flag.StringVar(&config.BackendPath, "backend_path", config.BackendPath,
		"Path prefix for backend requests (default: none)")
	flag.BoolVar(&config.PreserveHost, "preserve_host", config.PreserveHost,
		"Preserve Host header of the original request for "+
			"compatibility with cross-origin request checks.")
	flag.StringVar(&config.RelayScheme, "relay_scheme", config.RelayScheme,
		"Connection scheme (http, https) for connection from relay "+
			"client to relay server")
	flag.StringVar(&config.RelayAddress, "relay_address", config.RelayAddress,
		"Hostname of the relay server as seen by the relay client")
	flag.StringVar(&config.RelayPrefix, "relay_prefix", config.RelayPrefix,
		"Path prefix for the relay server")
	flag.StringVar(&config.ServerName, "server_name", config.ServerName,
		"Fetch requests from the relay server for this server name")
	flag.StringVar(&config.AuthenticationTokenFile, "authentication_token_file", config.AuthenticationTokenFile,
		"File with authentication token for backend requests")
	flag.StringVar(&config.RootCAFile, "root_ca_file", config.RootCAFile,
		"File with root CA cert for SSL")
	flag.IntVar(&config.MaxChunkSize, "max_chunk_size", config.MaxChunkSize,
		"Max size of data in bytes to accumulate before sending to the peer")
	flag.IntVar(&config.BlockSize, "block_size", config.BlockSize,
		"Size of i/o buffer in bytes")
	flag.IntVar(&config.NumPendingRequests, "num_pending_requests", config.NumPendingRequests,
		"Number of pending http requests to the relay")
	flag.IntVar(&config.MaxIdleConnsPerHost, "max_idle_conns_per_host", config.MaxIdleConnsPerHost,
		"The maximum number of idle (keep-alive) connections to keep per-host")
	flag.BoolVar(&config.DisableHttp2, "disable_http2", config.DisableHttp2,
		"Disable http2 protocol usage (e.g. for channels that use special streaming protocols such as SPDY).")
	flag.BoolVar(&config.ForceHttp2, "force_http2", config.ForceHttp2,
		"Force enable http2 protocol usage through the use of go's http2 transport (e.g. when relaying grpc).")
	flag.BoolVar(&config.DisableAuthForRemote, "disable_auth_for_remote", config.DisableAuthForRemote,
		"Disable auth when talking to the relay server for local testing.")

	// The stackdriver project ID is a client independent variable and so we
	// initialize it independently.
	flag.StringVar(&stackdriverProjectID, "trace-stackdriver-project-id", "",
		"If not empty, traces will be uploaded to this Google Cloud Project.")
}

func main() {
	flag.Parse()
	slog.SetDefault(slog.New(slog.NewJSONHandler(os.Stderr, nil)))

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

	client := client.NewClient(config)
	client.Start()
}
