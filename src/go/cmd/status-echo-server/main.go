// Copyright 2026 The Cloud Robotics Authors
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
	"flag"
	"fmt"
	"log/slog"
	"net/http"
	"os"

	"github.com/googlecloudrobotics/ilog"
)

var (
	port       = flag.Int("port", 80, "Port number to listen on")
	httpStatus = flag.Int("http_status", 200, "HTTP Status to echo")
	logLevel   = flag.Int("log_level", int(slog.LevelInfo), "the log message level required")
)

func echoHandler(w http.ResponseWriter, req *http.Request) {
	slog.Debug("echo", slog.String("path", req.URL.Path))
	w.Header().Set("Content-Type", "text/plain")
	w.WriteHeader(*httpStatus)
	w.Write([]byte(http.StatusText(*httpStatus)))
}

func main() {
	flag.Parse()

	// Setup logger
	logHandler := ilog.NewLogHandler(slog.Level(*logLevel), os.Stderr)
	slog.SetDefault(slog.New(logHandler))

	http.HandleFunc("/", echoHandler)

	addr := fmt.Sprintf(":%d", *port)
	slog.Info("Status echo server running", slog.String("address", addr))

	if err := http.ListenAndServe(addr, nil); err != nil {
		slog.Error("Server failed to start", slog.Any("error", err))
		os.Exit(1)
	}
}
