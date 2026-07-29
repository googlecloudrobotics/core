package main

import (
	"fmt"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"testing"
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

		h := newHandlers()
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

		h := newHandlers()
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

		h := newHandlers()
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
