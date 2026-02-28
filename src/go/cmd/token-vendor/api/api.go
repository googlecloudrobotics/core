// Copyright 2022 The Cloud Robotics Authors
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

package api

import (
	"log/slog"
	"net/http"
	"time"

	"github.com/prometheus/client_golang/prometheus/promhttp"
)

const (
	httpTimeoutRead    = 10 * time.Second
	httpTimeoutWrite   = 10 * time.Second
	httpTimeoutHandler = 10 * time.Second
)

type constHandler []byte

func (ch constHandler) ServeHTTP(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Metadata-Flavor", "Google")
	w.Header().Set("Content-Type", "application/text")
	w.WriteHeader(http.StatusOK)
	w.Write(ch)
}

// Register generic API API handlers functions to the default http.DefaultServeMux
func Register() error {
	http.Handle("/healthz", constHandler("ok"))
	http.Handle("/metrics", promhttp.Handler())
	return nil
}

// Setup and serve. Never returns. Handlers need to be registered before.
func SetupAndServe(addr string) error {
	srv := &http.Server{
		Addr:         addr,
		ReadTimeout:  httpTimeoutRead,
		WriteTimeout: httpTimeoutWrite,
		Handler:      LoggingMiddleware(http.DefaultServeMux),
	}
	slog.Info("API listening", slog.String("Address", addr))
	return srv.ListenAndServe()
}

func LoggingMiddleware(handler http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		xFwd := r.Header.Get("X-Forwarded-For")
		slog.Debug("Forwarding request",
			slog.String("RemoteAddr", r.RemoteAddr),
			slog.String("For", xFwd),
			slog.String("Method", r.Method),
			slog.String("URL", r.URL.String()))
		handler.ServeHTTP(w, r)
	})
}

func ErrResponse(w http.ResponseWriter, statusCode int, message string) {
	slog.Warn("Error response", slog.Int("Code", statusCode), slog.String("Message", message))
	w.WriteHeader(statusCode)
	w.Write([]byte(message))
}
