package main

import (
	"net/http"
	"net/http/httptest"
	"strconv"
	"strings"
	"testing"

	"github.com/prometheus/client_golang/prometheus/promhttp"
)

func getLegacyRequestCount(t *testing.T) int {
	t.Helper()
	req := httptest.NewRequest(http.MethodGet, "/metrics", nil)
	rec := httptest.NewRecorder()
	promhttp.Handler().ServeHTTP(rec, req)
	if rec.Code != http.StatusOK {
		t.Fatalf("expected status 200 OK from /metrics, got %d", rec.Code)
	}
	for _, line := range strings.Split(rec.Body.String(), "\n") {
		if strings.HasPrefix(line, "legacy_requests_total ") {
			valStr := strings.TrimPrefix(line, "legacy_requests_total ")
			val, err := strconv.Atoi(strings.TrimSpace(valStr))
			if err != nil {
				t.Fatalf("failed to parse counter value %q: %v", valStr, err)
			}
			return val
		}
	}
	return 0
}

func TestAuthLegacyCredentialsAccepted(t *testing.T) {
	oldAccept := *acceptLegacyCredentials
	*acceptLegacyCredentials = true
	defer func() { *acceptLegacyCredentials = oldAccept }()

	h := newHandlers()

	before := getLegacyRequestCount(t)

	req := httptest.NewRequest(http.MethodGet, "/auth", nil)
	req.Header.Set("Authorization", "Bearer ya29.test-legacy-token")
	rec := httptest.NewRecorder()

	h.auth(rec, req)

	if rec.Code != http.StatusOK {
		t.Fatalf("expected status 200 OK, got %d: %s", rec.Code, rec.Body.String())
	}
	if gotAuth := rec.Header().Get("Authorization"); gotAuth != "Bearer ya29.test-legacy-token" {
		t.Errorf("expected Authorization header preserved %q, got %q", "Bearer ya29.test-legacy-token", gotAuth)
	}

	after := getLegacyRequestCount(t)
	if after-before != 1 {
		t.Errorf("expected legacyRequests counter to increment by 1, before=%d, after=%d", before, after)
	}
}

func TestAuthLegacyCredentialsRejected(t *testing.T) {
	oldAccept := *acceptLegacyCredentials
	*acceptLegacyCredentials = false
	defer func() { *acceptLegacyCredentials = oldAccept }()

	h := newHandlers()

	before := getLegacyRequestCount(t)

	req := httptest.NewRequest(http.MethodGet, "/auth", nil)
	req.Header.Set("Authorization", "Bearer ya29.test-legacy-token")
	rec := httptest.NewRecorder()

	h.auth(rec, req)

	if rec.Code != http.StatusUnauthorized {
		t.Fatalf("expected status 401 Unauthorized, got %d: %s", rec.Code, rec.Body.String())
	}

	after := getLegacyRequestCount(t)
	if after != before {
		t.Errorf("expected legacyRequests counter not to increment, before=%d, after=%d", before, after)
	}
}

func TestMetricsEndpoint(t *testing.T) {
	oldAccept := *acceptLegacyCredentials
	*acceptLegacyCredentials = true
	defer func() { *acceptLegacyCredentials = oldAccept }()

	h := newHandlers()
	authReq := httptest.NewRequest(http.MethodGet, "/auth", nil)
	authReq.Header.Set("Authorization", "Bearer ya29.test-metrics-token")
	h.auth(httptest.NewRecorder(), authReq)

	req := httptest.NewRequest(http.MethodGet, "/metrics", nil)
	rec := httptest.NewRecorder()

	handler := promhttp.Handler()
	handler.ServeHTTP(rec, req)

	if rec.Code != http.StatusOK {
		t.Fatalf("expected status 200 OK, got %d", rec.Code)
	}

	body := rec.Body.String()
	if !strings.Contains(body, "legacy_requests_total") {
		t.Errorf("expected /metrics output to contain legacy_requests_total, got:\n%s", body)
	}
}
