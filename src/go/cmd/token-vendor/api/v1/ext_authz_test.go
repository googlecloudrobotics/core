package v1

import (
	"net/http"
	"net/http/httptest"
	"net/url"
	"path"
	"testing"
)

// TestExtAuthz_TokenVerify_Methods verifies that token.verify handles subrequests
// with various HTTP verbs (GET, POST, PUT, DELETE) when AllowAnyMethod is enabled,
// and enforces method restrictions when AllowAnyMethod is disabled.
func TestExtAuthz_TokenVerify_Methods(t *testing.T) {
	t.Run("AllowAnyMethod_True", func(t *testing.T) {
		opts := &Options{AllowAnyMethod: true}
		h := NewHandlerContext(nil, opts)
		mux := http.NewServeMux()

		prefix := "/apis/core.token-vendor/v1"
		verifyPrefix := ""
		if !opts.AllowAnyMethod {
			verifyPrefix = "GET "
		}
		mux.HandleFunc(verifyPrefix+path.Join(prefix, "token.verify"), h.verifyTokenHandler)

		methods := []string{http.MethodGet, http.MethodPost, http.MethodPut, http.MethodDelete}

		for _, method := range methods {
			t.Run("Method_"+method, func(t *testing.T) {
				req := httptest.NewRequest(method, "/apis/core.token-vendor/v1/token.verify", nil)
				req.Header.Set("x-crc-tv-robots", "true")
				w := httptest.NewRecorder()

				mux.ServeHTTP(w, req)

				if w.Code == http.StatusMethodNotAllowed || w.Code == http.StatusNotFound {
					t.Fatalf("unexpected status %d for HTTP method %s", w.Code, method)
				}
				if w.Code != http.StatusBadRequest {
					t.Errorf("got status %d, want %d", w.Code, http.StatusBadRequest)
				}
			})
		}
	})

	t.Run("AllowAnyMethod_False", func(t *testing.T) {
		opts := &Options{AllowAnyMethod: false}
		h := NewHandlerContext(nil, opts)
		mux := http.NewServeMux()

		prefix := "/apis/core.token-vendor/v1"
		verifyPrefix := ""
		if !opts.AllowAnyMethod {
			verifyPrefix = "GET "
		}
		mux.HandleFunc(verifyPrefix+path.Join(prefix, "token.verify"), h.verifyTokenHandler)

		req := httptest.NewRequest(http.MethodPost, "/apis/core.token-vendor/v1/token.verify", nil)
		req.Header.Set("x-crc-tv-robots", "true")
		w := httptest.NewRecorder()

		mux.ServeHTTP(w, req)

		if w.Code != http.StatusMethodNotAllowed {
			t.Errorf("got status %d, want %d for POST when AllowAnyMethod is false", w.Code, http.StatusMethodNotAllowed)
		}
	})
}

// TestExtAuthz_TokenVerify_RobotVsHumanHeader directly tests testForRobotACL header and query parameters.
func TestExtAuthz_TokenVerify_RobotVsHumanHeader(t *testing.T) {
	t.Run("RobotProviderHeader", func(t *testing.T) {
		u, err := url.Parse("http://localhost/apis/core.token-vendor/v1/token.verify")
		if err != nil {
			t.Fatalf("failed to parse URL: %v", err)
		}
		hdr := http.Header{"X-Crc-Tv-Robots": []string{"true"}}

		robots, err := testForRobotACL(u, &hdr)
		if err != nil {
			t.Fatalf("unexpected error: %v", err)
		}
		if !robots {
			t.Errorf("got robots = %v, want true", robots)
		}
	})

	t.Run("HumanProviderHeader", func(t *testing.T) {
		u, err := url.Parse("http://localhost/apis/core.token-vendor/v1/token.verify")
		if err != nil {
			t.Fatalf("failed to parse URL: %v", err)
		}
		hdr := http.Header{"X-Crc-Tv-Robots": []string{"false"}}

		robots, err := testForRobotACL(u, &hdr)
		if err != nil {
			t.Fatalf("unexpected error: %v", err)
		}
		if robots {
			t.Errorf("got robots = %v, want false", robots)
		}
	})

	t.Run("ConflictHeaderAndQuery", func(t *testing.T) {
		u, err := url.Parse("http://localhost/apis/core.token-vendor/v1/token.verify?robots=false")
		if err != nil {
			t.Fatalf("failed to parse URL: %v", err)
		}
		hdr := http.Header{"X-Crc-Tv-Robots": []string{"true"}}

		_, err = testForRobotACL(u, &hdr)
		if err == nil {
			t.Errorf("expected error when both header and query param are set, got nil")
		}
	})
}
