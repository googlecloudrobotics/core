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

// Package main runs a simple HTTP server that echoes back a configured
// or requested HTTP status code.
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
	"fmt"
	"io"
	"log/slog"
	"net"
	"net/http"
	"os"
	"os/signal"
	"strconv"
	"strings"
	"sync/atomic"
	"syscall"
	"time"

	"github.com/googlecloudrobotics/ilog"
)

const (
	shutdownPeriod      = 15 * time.Second
	readinessDrainDelay = 5 * time.Second
)

var (
	port           = flag.Int("port", 80, "Port number to listen on")
	httpStatus     = flag.Int("http_status", 200, "Default HTTP Status to echo")
	logLevel       = flag.Int("log_level", int(slog.LevelInfo), "the log message level required")
	isShuttingDown atomic.Bool
)

func echoHandler(w http.ResponseWriter, req *http.Request) {
	slog.Debug("echo", slog.String("path", req.URL.Path))
	w.Header().Set("Content-Type", "text/plain")

	statusCode := determineStatusCode(req, *httpStatus)
	w.WriteHeader(statusCode)
	io.WriteString(w, http.StatusText(statusCode))
}

func determineStatusCode(req *http.Request, defaultStatus int) int {
	// Default to the status code configured via flags.
	statusCode := defaultStatus
	statusStr := ""
	path := req.URL.Path

	// Try to extract status code from path suffix (e.g., /status/200) first, falling back to the "status" query parameter if not found.
	if idx := strings.LastIndex(path, "/status/"); idx != -1 {
		suffix := path[idx+len("/status/"):]
		// Ensure there are no more slashes after "/status/" to guarantee the pattern is strictly .../status/<something>$
		if !strings.Contains(suffix, "/") {
			statusStr = suffix
		}
	}
	if (statusStr == "") && (req.URL.RawQuery != "") {
		statusStr = req.URL.Query().Get("status")
	}

	// If a status code was requested, validate and parse it. Falls back to 400 Bad Request on error.
	if statusStr != "" {
		parsedCode, err := strconv.Atoi(statusStr)
		if err != nil {
			slog.Warn("Status code could not be converted to integer", "error", err)
			statusCode = http.StatusBadRequest
		} else if http.StatusText(parsedCode) == "" {
			slog.Warn("Unrecognized HTTP status code requested", "code", parsedCode)
			statusCode = http.StatusBadRequest
		} else {
			statusCode = parsedCode
		}
	}

	return statusCode
}

func healthzHandler(w http.ResponseWriter, req *http.Request) {
	io.WriteString(w, "OK\n")
}

func readyzHandler(w http.ResponseWriter, req *http.Request) {
	if isShuttingDown.Load() {
		http.Error(w, "Server shutting down", http.StatusServiceUnavailable)
		return
	}
	io.WriteString(w, "OK\n")
}

func main() {
	flag.Parse()
	ctx, stop := signal.NotifyContext(context.Background(), syscall.SIGINT, syscall.SIGTERM)
	defer stop()

	// Setup logger
	logHandler := ilog.NewLogHandler(slog.Level(*logLevel), os.Stderr)
	slog.SetDefault(slog.New(logHandler))

	mux := http.NewServeMux()
	mux.HandleFunc("/healthz", healthzHandler)
	mux.HandleFunc("/readyz", readyzHandler)
	mux.HandleFunc("/", echoHandler)

	addr := fmt.Sprintf(":%d", *port)

	ongoingCtx, stopOngoingGracefully := context.WithCancel(context.Background())
	srv := &http.Server{
		Addr:         addr,
		Handler:      mux,
		ReadTimeout:  1 * time.Second,
		WriteTimeout: 2 * time.Second,
		IdleTimeout:  10 * time.Second,
		BaseContext: func(_ net.Listener) context.Context {
			return ongoingCtx
		},
	}

	go func() {
		if err := srv.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			slog.Error("Server failed to start", slog.Any("error", err))
			os.Exit(1)
		}
	}()

	<-ctx.Done()
	// Stop listening for OS signals and handoff to graceful degradation.
	stop()
	shutdownServer(srv, stopOngoingGracefully)
}

func shutdownServer(srv *http.Server, forceCancel context.CancelFunc) {
	// Fail health checks to stop routing of new traffic to terminating pod.
	isShuttingDown.Store(true)
	slog.Info("Received shutdown signal. Failing health check and waiting for drain delay.")
	time.Sleep(readinessDrainDelay)

	// Attempt a graceful shutdown (finish processing of current requests).
	slog.Info("Initiating server shutdown", "timeout", shutdownPeriod)
	shutdownCtx, cancel := context.WithTimeout(context.Background(), shutdownPeriod)
	defer cancel()

	err := srv.Shutdown(shutdownCtx)

	// Cancel base context to signal any stubborn handlers to abort.
	forceCancel()

	// Hard stop after graceful shutdown time out.
	if err != nil {
		slog.Warn("Failed to wait for ongoing requests to finish, forcing cancellation.", "error", err)
		srv.Close()
		return
	}

	slog.Info("Server has shut down gracefully")

}
