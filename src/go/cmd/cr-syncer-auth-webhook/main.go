// Copyright 2025 The Cloud Robotics Authors
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

// The cr-syncer-auth-webhook verifies that requests from the cr-syncer are
// limited to the robot named in the credentials.
package main

import (
	"context"
	"flag"
	"fmt"
	"log/slog"
	"net/http"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/pkg/errors"

	"github.com/googlecloudrobotics/ilog"
)

var (
	port = flag.Int("port", 8080,
		"Listening port for HTTP requests")

	acceptLegacyCredentials = flag.Bool("accept-legacy-service-account-credentials", false,
		"Whether to accept legacy GCP service account credentials")

	tokenVendor = flag.String("token-vendor", "http://token-vendor.app-token-vendor.svc.cluster.local",
		"Hostname of the token-vendor service")

	logLevel = flag.Int("log-level", int(slog.LevelInfo),
		"the log message level required to be logged")
)

type handlers struct {
	client *http.Client
}

func newHandlers() handlers {
	return handlers{
		client: &http.Client{},
	}
}

func (h *handlers) health(w http.ResponseWriter, r *http.Request) {
	w.WriteHeader(http.StatusOK)
}

func (h *handlers) auth(w http.ResponseWriter, r *http.Request) {
	// TODO(rodrigoq): check for JWT and compare to request pattern
	if *acceptLegacyCredentials {
		// The request already has the necessary credentials, so preserve these.
		w.Header().Add("Authorization", r.Header.Get("Authorization"))
		w.WriteHeader(http.StatusOK)
		return
	}
	http.Error(w, "No valid credentials provided", http.StatusUnauthorized)
}

func main() {
	flag.Parse()
	logHandler := ilog.NewLogHandler(slog.Level(*logLevel), os.Stderr)
	slog.SetDefault(slog.New(logHandler))

	server := &http.Server{
		Addr: fmt.Sprintf(":%d", *port),
	}
	handlers := newHandlers()
	http.HandleFunc("/healthz", handlers.health)
	http.HandleFunc("/auth", handlers.auth)

	go func() {
		slog.Info("Serving requests...")
		if err := server.ListenAndServe(); !errors.Is(err, http.ErrServerClosed) {
			slog.Error("server.ListenAndServe() failed unexpectedly", ilog.Err(err))
			os.Exit(1)
		}
		slog.Info("Stopped serving new connections.")
	}()

	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, syscall.SIGINT, syscall.SIGTERM)
	<-sigChan

	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()

	if err := server.Shutdown(ctx); err != nil {
		slog.Error("server.Shutdown() failed unexpectedly", ilog.Err(err))
		os.Exit(1)
	}
	slog.Info("Server shutdown complete.")
}
