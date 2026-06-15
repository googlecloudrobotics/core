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

package main

import (
	"net/http"
	"net/http/httptest"
	"testing"
)

func TestEchoHandler(t *testing.T) {
	tests := []struct {
		desc       string
		path       string
		httpStatus int
		wantStatus int
		wantBody   string
	}{
		{
			desc:       "OK status on root path",
			path:       "/",
			httpStatus: http.StatusOK,
			wantStatus: http.StatusOK,
			wantBody:   "OK",
		},
		{
			desc:       "Internal Server Error on nested path",
			path:       "/foo/bar?key=value",
			httpStatus: http.StatusInternalServerError,
			wantStatus: http.StatusInternalServerError,
			wantBody:   "Internal Server Error",
		},
		{
			desc:       "OK status on deep path",
			path:       "/a/b/c/d/e",
			httpStatus: http.StatusOK,
			wantStatus: http.StatusOK,
			wantBody:   "OK",
		},
		{
			desc:       "Status from path",
			path:       "/status/201",
			httpStatus: http.StatusOK,
			wantStatus: http.StatusCreated,
			wantBody:   "Created",
		},
		{
			desc:       "Status from query",
			path:       "/?status=202",
			httpStatus: http.StatusOK,
			wantStatus: http.StatusAccepted,
			wantBody:   "Accepted",
		},
		{
			desc:       "Invalid status in path defaults to 400",
			path:       "/status/999",
			httpStatus: http.StatusOK,
			wantStatus: http.StatusBadRequest,
			wantBody:   "Bad Request",
		},
		{
			desc:       "Non-numeric status in query defaults to 400",
			path:       "/?status=abc",
			httpStatus: http.StatusOK,
			wantStatus: http.StatusBadRequest,
			wantBody:   "Bad Request",
		},
		{
			desc:       "Path status takes precedence over query",
			path:       "/status/201?status=202",
			httpStatus: http.StatusOK,
			wantStatus: http.StatusCreated,
			wantBody:   "Created",
		},
	}

	for _, tc := range tests {
		t.Run(tc.desc, func(t *testing.T) {
			*httpStatus = tc.httpStatus

			req := httptest.NewRequest(http.MethodGet, tc.path, nil)
			w := httptest.NewRecorder()

			echoHandler(w, req)

			resp := w.Result()
			if resp.StatusCode != tc.wantStatus {
				t.Errorf("got %d, want %d", resp.StatusCode, tc.wantStatus)
			}

			contentType := resp.Header.Get("Content-Type")
			if contentType != "text/plain" {
				t.Errorf("got %q, want %q", contentType, "text/plain")
			}

			body := w.Body.String()
			if body != tc.wantBody {
				t.Errorf("got %q, want %q", body, tc.wantBody)
			}
		})
	}
}

type discardResponseWriter struct {
	header http.Header
}

func newDiscardResponseWriter() *discardResponseWriter {
	return &discardResponseWriter{
		header: make(http.Header),
	}
}

func (d *discardResponseWriter) Header() http.Header {
	return d.header
}

func (d *discardResponseWriter) Write(b []byte) (int, error) {
	return len(b), nil
}

func (d *discardResponseWriter) WriteHeader(statusCode int) {
}

func BenchmarkEchoHandler(b *testing.B) {
	req := httptest.NewRequest(http.MethodGet, "/status/200", nil)
	w := newDiscardResponseWriter()
	for b.Loop() {
		echoHandler(w, req)
	}
}

func TestHealthzHandler(t *testing.T) {
	req := httptest.NewRequest(http.MethodGet, "/healthz", nil)
	w := httptest.NewRecorder()
	healthzHandler(w, req)
	resp := w.Result()
	if resp.StatusCode != http.StatusOK {
		t.Errorf("got %d, want %d", resp.StatusCode, http.StatusOK)
	}
	body := w.Body.String()
	if body != "OK\n" {
		t.Errorf("got %q, want %q", body, "OK\n")
	}
}

func TestReadyzHandler(t *testing.T) {
	req := httptest.NewRequest(http.MethodGet, "/readyz", nil)
	w := httptest.NewRecorder()
	readyzHandler(w, req)
	resp := w.Result()
	if resp.StatusCode != http.StatusOK {
		t.Errorf("got %d, want %d", resp.StatusCode, http.StatusOK)
	}
	body := w.Body.String()
	if body != "OK\n" {
		t.Errorf("got %q, want %q", body, "OK\n")
	}
}

func TestReadyzHandler_ShuttingDown(t *testing.T) {
	isShuttingDown.Store(true)
	defer isShuttingDown.Store(false) // Reset after test

	req := httptest.NewRequest(http.MethodGet, "/readyz", nil)
	w := httptest.NewRecorder()
	readyzHandler(w, req)
	resp := w.Result()

	if resp.StatusCode != http.StatusServiceUnavailable {
		t.Errorf("got %d, want %d", resp.StatusCode, http.StatusServiceUnavailable)
	}
}
