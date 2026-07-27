package main

import (
	"net/http"
	"net/http/httptest"
	"testing"
)

// TestExtAuthz_CRSyncer_HeaderFallback tests that extractOriginalURL correctly
// falls back to X-Envoy-Original-Path when X-Original-Url is not present.
func TestExtAuthz_CRSyncer_HeaderFallback(t *testing.T) {
	tests := []struct {
		name       string
		headers    map[string]string
		wantURLStr string
	}{
		{
			name: "EnvoyOriginalPathHeader",
			headers: map[string]string{
				"X-Envoy-Original-Path": "/apis/core.cloudrobotics.com/v1alpha1/namespaces/default/robots",
			},
			wantURLStr: "/apis/core.cloudrobotics.com/v1alpha1/namespaces/default/robots",
		},
		{
			name: "LegacyOriginalUrlHeaderPriority",
			headers: map[string]string{
				"X-Original-Url":        "/apis/legacy/path",
				"X-Envoy-Original-Path": "/apis/envoy/path",
			},
			wantURLStr: "/apis/legacy/path",
		},
	}

	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			req := httptest.NewRequest(http.MethodPost, "/auth", nil)
			for k, v := range tc.headers {
				req.Header.Set(k, v)
			}

			got := extractOriginalURL(req)
			if got != tc.wantURLStr {
				t.Errorf("extractOriginalURL() = %q, want %q", got, tc.wantURLStr)
			}
		})
	}
}
