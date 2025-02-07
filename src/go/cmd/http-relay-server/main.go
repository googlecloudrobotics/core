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

// Package main runs a multiplexing HTTP relay server.
//
// It exists to make HTTP endpoints on robots accessible without a public
// endpoint. It binds to a public endpoint accessible by both user-client and
// backend and works together with a relay-client that's colocated with the
// backend.
//
//	  lan        |    internet      |               lan
//	             |                  |
//	user-client ---> relay server <--- relay-client ---> backend
//	             |                  |
//	         firewall           firewall
//
// The relay server is multiplexing: It allows multiple relay-clients to
// connect under unique names, each handling requests for a subpath of /client.
// Alternatively (e.g. for grpc conenctions) the backend can be selected by
// omitting the client prefix and passing an `X-Server-Name` header.
//
// Sequence of operations:
//   - Web-client makes request on /client/$foo/$request.
//   - Relay server assigns an ID and stores request (with path $request) in
//     memory. It keeps the user-client's request pending.
//   - Relay-client requests /server/request?server=$foo
//   - Relay server responds with stored request (or timeout if no request comes
//     in within the next 30 sec).
//   - Relay-client makes the stored request to backend.
//   - Backend replies.
//   - Relay-client posts backend's reply to /server/response.
//   - Relay server responds to client's request with backend's reply.
//
// For some requests (eg kubectl exec), the backend responds with
// 101 Switching Protocols, resulting in the following operations.
//   - Relay server responds to client's request with backend's 101 reply.
//   - Client sends bytes from stdin to the relay server.
//   - Relay-client requests /server/requeststream?id=$id.
//   - Relay server responds with stdin bytes from client.
//   - Relay-client sends stdin bytes to backend.
//   - Backend sends stdout bytes to relay-client.
//   - Relay-client posts stdout bytes to /server/response.
//   - Relay server sends stdout bytes to the client.
//
// This simplified graphic shows the back-and-forth for an `exec` request:
//
//	user-client ---> relay server <--- relay-client ---> backend
//	      .     |        .         |       .               .
//	      . -POST /exec->.         |       .               .
//	      .     |        . <-GET /request- .               .
//	      .     |        . ---- exec ----> .               .
//	      .     |        .         |       . -POST /exec-> .
//	      .     |        .         |       . <--- 101 ---- .
//	      .     |        .<-POST /response-.               .
//	      . <-- 101 ---- .         |       .               .
//	      . -- stdin --> .         |       .               .
//	      .     |        .<-POST /request- .               .
//	      .     |        .        stream   .               .
//	      .     |        . ---- stdin ---> .               .
//	      .     |        .         |       . --- stdin --> .
//	      .     |        .         |       . <-- stdout--- .
//	      .     |        .<-POST /response-.               .
//	      . <- stdout -- .         |       .               .
//	      .     |        .         |       .               .
//
// The relay-client side implementation is in ../http-relay-client.
package main

import (
	"flag"
	"log/slog"
	"os"

	"contrib.go.opencensus.io/exporter/stackdriver"
	"github.com/googlecloudrobotics/core/src/go/cmd/http-relay-server/server"
	"github.com/googlecloudrobotics/ilog"
	"go.opencensus.io/trace"
)

var (
	port      = flag.Int("port", 80, "Port number to listen on")
	blockSize = flag.Int("block_size", 10*1024,
		"Size of i/o buffer in bytes")
	stackdriverProjectID = flag.String("trace-stackdriver-project-id", "",
		"If not empty, traces will be uploaded to this Google Cloud Project.")
	logLevel = flag.Int("log_level", int(slog.LevelInfo),
		"the log message level required to be logged")
)

func main() {
	flag.Parse()
	logHandler := ilog.NewLogHandler(slog.Level(*logLevel), os.Stderr)
	slog.SetDefault(slog.New(logHandler))

	if *stackdriverProjectID != "" {
		sd, err := stackdriver.NewExporter(stackdriver.Options{
			ProjectID: *stackdriverProjectID,
		})
		if err != nil {
			slog.Error("Failed to create the Stackdriver exporter", slog.String("Project", *stackdriverProjectID), ilog.Err(err))
			os.Exit(1)
		} else {
			trace.RegisterExporter(sd)
			defer sd.Flush()
		}
	}

	server := server.NewServer()
	server.Start(*port, *blockSize)
}
