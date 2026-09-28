package main

import (
	"fmt"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"testing"

	"github.com/prometheus/client_golang/prometheus/promhttp"
)

// testValidJWT is a base64-encoded unencrypted JWT header.payload.signature
// payload: {"sub":"robot-1","iss":"test"}
const testValidJWT = "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiJyb2JvdC0xIiwiaXNzIjoidGVzdCJ9.c2ln"

func TestProxyKubernetes(t *testing.T) {
	t.Run("unauthorized", func(t *testing.T) {
		// Mock token vendor returning 403 Forbidden for invalid JWT
		tokenVendorServer := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			w.WriteHeader(http.StatusForbidden)
		}))
		defer tokenVendorServer.Close()

		oldTokenVendor := *tokenVendor
		*tokenVendor = tokenVendorServer.URL
		defer func() { *tokenVendor = oldTokenVendor }()

		h, err := newHandlers()
		if err != nil {
			t.Fatalf("newHandlers() failed: %v", err)
		}
		req := httptest.NewRequest(http.MethodGet, "/apis/core.kubernetes/apis/registry.cloudrobotics.com/v1alpha1/namespaces/default/robots/robot-1", nil)
		req.Header.Set("Authorization", "Bearer invalid-jwt")
		rec := httptest.NewRecorder()

		h.proxyKubernetes(rec, req)

		if rec.Code != http.StatusUnauthorized {
			t.Errorf("proxyKubernetes status = %d, want %d", rec.Code, http.StatusUnauthorized)
		}
	})

	t.Run("forbidden", func(t *testing.T) {
		// Mock token vendor returning 200 OK for valid JWT
		tokenVendorServer := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			w.WriteHeader(http.StatusOK)
		}))
		defer tokenVendorServer.Close()

		oldTokenVendor := *tokenVendor
		*tokenVendor = tokenVendorServer.URL
		defer func() { *tokenVendor = oldTokenVendor }()

		h, err := newHandlers()
		if err != nil {
			t.Fatalf("newHandlers() failed: %v", err)
		}
		// Request attempts to access robot-2 resource with robot-1 credentials
		req := httptest.NewRequest(http.MethodGet, "/apis/core.kubernetes/apis/registry.cloudrobotics.com/v1alpha1/namespaces/default/robots/robot-2", nil)
		req.Header.Set("Authorization", "Bearer "+testValidJWT)
		rec := httptest.NewRecorder()

		h.proxyKubernetes(rec, req)

		if rec.Code != http.StatusForbidden {
			t.Errorf("proxyKubernetes status = %d, want %d", rec.Code, http.StatusForbidden)
		}
	})

	t.Run("success", func(t *testing.T) {
		// Mock token vendor returning 200 OK
		tokenVendorServer := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			w.WriteHeader(http.StatusOK)
		}))
		defer tokenVendorServer.Close()

		// Mock Kubernetes API server backend
		gotPath := ""
		gotAuthHeader := ""
		k8sBackend := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			gotPath = r.URL.Path
			gotAuthHeader = r.Header.Get("Authorization")
			w.WriteHeader(http.StatusOK)
			fmt.Fprint(w, `{"kind":"RobotList"}`)
		}))
		defer k8sBackend.Close()

		// Create temp SA token file
		tmpDir := t.TempDir()
		tokenFile := filepath.Join(tmpDir, "token")
		if err := os.WriteFile(tokenFile, []byte("mock-k8s-sa-token"), 0600); err != nil {
			t.Fatalf("failed to write mock token file: %v", err)
		}

		oldTokenVendor := *tokenVendor
		oldK8sTarget := *k8sTarget
		oldK8sTokenPath := *k8sTokenPath
		*tokenVendor = tokenVendorServer.URL
		*k8sTarget = k8sBackend.URL
		*k8sTokenPath = tokenFile
		defer func() {
			*tokenVendor = oldTokenVendor
			*k8sTarget = oldK8sTarget
			*k8sTokenPath = oldK8sTokenPath
		}()

		h, err := newHandlers()
		if err != nil {
			t.Fatalf("newHandlers() failed: %v", err)
		}
		req := httptest.NewRequest(http.MethodGet, "/apis/core.kubernetes/apis/registry.cloudrobotics.com/v1alpha1/namespaces/default/robots/robot-1", nil)
		req.Header.Set("Authorization", "Bearer "+testValidJWT)
		rec := httptest.NewRecorder()

		h.proxyKubernetes(rec, req)

		if rec.Code != http.StatusOK {
			t.Errorf("proxyKubernetes status = %d, want %d", rec.Code, http.StatusOK)
		}
		wantPath := "/apis/registry.cloudrobotics.com/v1alpha1/namespaces/default/robots/robot-1"
		if gotPath != wantPath {
			t.Errorf("proxied path = %q, want %q", gotPath, wantPath)
		}
		wantAuth := "Bearer mock-k8s-sa-token"
		if gotAuthHeader != wantAuth {
			t.Errorf("proxied Authorization = %q, want %q", gotAuthHeader, wantAuth)
		}
	})
}

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

	h, err := newHandlers()
	if err != nil {
		t.Fatalf("newHandlers() failed: %v", err)
	}

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

	h, err := newHandlers()
	if err != nil {
		t.Fatalf("newHandlers() failed: %v", err)
	}

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

	h, err := newHandlers()
	if err != nil {
		t.Fatalf("newHandlers() failed: %v", err)
	}
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
