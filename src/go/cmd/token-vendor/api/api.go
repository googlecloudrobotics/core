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
	"net/http"
	"time"

	"github.com/prometheus/client_golang/prometheus/promhttp"
	log "github.com/sirupsen/logrus"
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
		Handler: http.TimeoutHandler(LoggingMiddleware(http.DefaultServeMux),
			httpTimeoutHandler, "handler timeout"),
	}
	log.Info("api listening on ", addr)
	return srv.ListenAndServe()
}

func LoggingMiddleware(handler http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		xFwd := r.Header.Get("X-Forwarded-For")
		log.Debugf("[%s, for %q] [%s] %s", r.RemoteAddr, xFwd, r.Method, r.URL)
		handler.ServeHTTP(w, r)
	})
}

func ErrResponse(w http.ResponseWriter, statusCode int, message string) {
	log.Warnf("-> error [%d] %s", statusCode, message)
	w.WriteHeader(statusCode)
	w.Write([]byte(message))
}
