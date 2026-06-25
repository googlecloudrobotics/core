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
// For more details, see README.md.
package main

import (
	"context"
	"flag"
	"log/slog"
	"os"
	"os/signal"
	"syscall"

	"contrib.go.opencensus.io/exporter/stackdriver"
	"github.com/googlecloudrobotics/core/src/go/cmd/http-relay-server/server"
	"github.com/googlecloudrobotics/ilog"
	"go.opencensus.io/trace"
)

var (
	port      = flag.Int("port", server.DefaultPort, "Port number to listen on")
	blockSize = flag.Int("block_size", server.DefaultBlockSize,
		"Size of i/o buffer in bytes")
	stackdriverProjectID = flag.String("trace-stackdriver-project-id", "",
		"If not empty, traces will be uploaded to this Google Cloud Project.")
	logLevel = flag.Int("log_level", int(slog.LevelInfo),
		"the log message level required to be logged")
	inactiveRequestTimeout = flag.Duration("inactive_request_timeout", server.DefaultInactiveRequestTimeout,
		"Timeout for inactive requests. In particular, this sets a limit on how long the backend can wait before writing headers and the response status.")
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

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	server := server.NewServer(server.Config{
		Port:                   *port,
		BlockSize:              *blockSize,
		InactiveRequestTimeout: *inactiveRequestTimeout,
	})
	server.Start(ctx)
}
