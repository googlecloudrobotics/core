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
		mux.HandleFunc(path.Join(prefix, "token.verify"), h.verifyTokenHandler)

		methods := []string{http.MethodGet, http.MethodPost, http.MethodPut, http.MethodDelete}

		for _, method := range methods {
			t.Run("Method_"+method, func(t *testing.T) {
				req := httptest.NewRequest(method, "/apis/core.token-vendor/v1/token.verify", nil)
				req.Header.Set(headerRobots, "true")
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
		req.Header.Set(headerRobots, "true")
		w := httptest.NewRecorder()

		mux.ServeHTTP(w, req)

		if w.Code != http.StatusMethodNotAllowed {
			t.Errorf("got status %d, want %d for POST when AllowAnyMethod is false", w.Code, http.StatusMethodNotAllowed)
		}
	})
}

// TestExtAuthz_TokenVerify_RobotVsHumanHeader directly tests testForRobotACL header and query parameters.
func TestExtAuthz_TokenVerify_RobotVsHumanHeader(t *testing.T) {
	tests := []struct {
		name       string
		rawURL     string
		headerVal  string
		wantRobots bool
		wantErr    bool
	}{
		{
			name:       "RobotProviderHeader",
			rawURL:     "http://localhost/apis/core.token-vendor/v1/token.verify",
			headerVal:  "true",
			wantRobots: true,
		},
		{
			name:       "HumanProviderHeader",
			rawURL:     "http://localhost/apis/core.token-vendor/v1/token.verify",
			headerVal:  "false",
			wantRobots: false,
		},
		{
			name:      "ConflictHeaderAndQuery",
			rawURL:    "http://localhost/apis/core.token-vendor/v1/token.verify?robots=false",
			headerVal: "true",
			wantErr:   true,
		},
	}

	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			u, err := url.Parse(tc.rawURL)
			if err != nil {
				t.Fatalf("failed to parse URL: %v", err)
			}
			var hdr http.Header
			if tc.headerVal != "" {
				hdr = make(http.Header)
				hdr.Set(headerRobots, tc.headerVal)
			}

			robots, err := testForRobotACL(u, &hdr)
			if tc.wantErr {
				if err == nil {
					t.Errorf("expected error, got nil")
				}
				return
			}
			if err != nil {
				t.Fatalf("unexpected error: %v", err)
			}
			if robots != tc.wantRobots {
				t.Errorf("got robots = %v, want %v", robots, tc.wantRobots)
			}
		})
	}
}
