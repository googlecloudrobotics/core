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
	"crypto/tls"
	"crypto/x509"
	"errors"
	"flag"
	"fmt"
	"io"
	"log/slog"
	"net/http"
	"net/http/httputil"
	"net/url"
	"os"
	"os/signal"
	"strings"
	"syscall"
	"time"

	"golang.org/x/oauth2/jws"

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

	k8sTarget = flag.String("k8s-target", "https://kubernetes.default.svc:443",
		"Target URL for Kubernetes API server reverse proxy")

	k8sTokenPath = flag.String("k8s-token-path", "/var/run/secrets/kubernetes.io/serviceaccount/token",
		"Path to Kubernetes ServiceAccount token")

	k8sCAPath = flag.String("k8s-ca-path", "/var/run/secrets/kubernetes.io/serviceaccount/ca.crt",
		"Path to Kubernetes CA certificate")
)

const (
	verifyJWTEndpoint = "/apis/core.token-vendor/v1/jwt.verify"

	legacyTokenPrefix = "ya29."
)

type handlers struct {
	client       *http.Client
	k8sTransport http.RoundTripper
	k8sTargetURL *url.URL
}

func newHandlers() *handlers {
	// Why: Cloning http.DefaultTransport preserves tuned connection pool defaults
	// (TCP keep-alives, ALPN negotiation, MaxIdleConnsPerHost) while allowing us
	// to attach custom TLS Root CAs for the internal K8s API server target.
	transport := http.DefaultTransport.(*http.Transport).Clone()
	if caCert, err := os.ReadFile(*k8sCAPath); err == nil {
		certPool := x509.NewCertPool()
		certPool.AppendCertsFromPEM(caCert)
		transport.TLSClientConfig = &tls.Config{
			RootCAs: certPool,
		}
	}

	targetURL, err := url.Parse(*k8sTarget)
	if err != nil {
		slog.Error("failed to parse k8s target URL", ilog.Err(err))
		os.Exit(1)
	}

	return &handlers{
		client:       &http.Client{},
		k8sTransport: transport,
		k8sTargetURL: targetURL,
	}
}

func (h *handlers) health(w http.ResponseWriter, r *http.Request) {
	w.WriteHeader(http.StatusOK)
}

// verifyJWT delegates to the token-vendor to verify the signature of the JWT
// matches the public key of the robot.
func (h *handlers) verifyJWT(encodedJWT string) error {
	if strings.HasPrefix(encodedJWT, legacyTokenPrefix) {
		// We can avoid the unnecessary request when the client is using a GCP
		// access token.
		return errors.New("legacy token format")
	}

	req, err := http.NewRequest("GET", *tokenVendor+verifyJWTEndpoint, nil)
	if err != nil {
		return fmt.Errorf("create request: %w", err)
	}
	req.Header.Add("Authorization", "Bearer "+encodedJWT)
	resp, err := h.client.Do(req)
	if err != nil {
		return fmt.Errorf("do request: %w", err)
	}
	defer resp.Body.Close()
	// Discard body so connection can be reused.
	io.Copy(io.Discard, resp.Body)

	if resp.StatusCode == http.StatusForbidden {
		return errors.New("invalid JWT")
	} else if resp.StatusCode != http.StatusOK {
		slog.Warn("unexpected status code from /jwt.verify", slog.Int("status", resp.StatusCode))
		return errors.New("unexpected status code")
	}
	return nil
}

func (h *handlers) resourceIsFiltered(groupKind string) bool {
	// TODO(rodrigoq): limit to CRDs with filter-by-robot-name label in case someone adds
	// new unfiltered resources in future.
	return groupKind != "registry.cloudrobotics.com/robottypes"
}

// validateRequestPath checks that the target URL path is expected for the cr-syncer and
// only accesses allowed resources.
func (h *handlers) validateRequestPath(urlString string, robotName string) error {
	incomingReq, err := parseURL(urlString)
	if err != nil {
		slog.Error("unexpected value of target URL path",
			slog.String("url", urlString), ilog.Err(err))
		return err
	}

	if !h.resourceIsFiltered(incomingReq.GroupKind) {
		// Unfiltered resources (eg robottypes) are always allowed.
		//
		// For additional defense-in-depth, we could check if the CRD has
		// annotations for the cr-syncer. However, the RBAC policy in
		// cr-syncer-policy.yaml already limits the client to syncable resources.
		return nil
	}

	// TODO(rodrigoq): check against label of upstream resource instead of assuming that
	// robot xyz can access all syncable resources matching *xyz.
	if incomingReq.RobotName != robotName && !strings.HasSuffix(incomingReq.ResourceName, robotName) {
		slog.Error("robot impersonation rejected",
			slog.String("sourceName", robotName),
			slog.String("targetName", incomingReq.RobotName+incomingReq.ResourceName),
			slog.String("kind", incomingReq.GroupKind),
			slog.String("url", urlString),
		)
		return errors.New("credentials rejected")
	}
	return nil
}

// proxyKubernetes reverse proxies requests under /apis/core.kubernetes/ to the
// Kubernetes API server after validating the client's credentials.
// Why: In Gateway API, Envoy's ExtAuthz filter appends response headers to client
// headers by default (causing duplicate Authorization header collisions). Serving
// /apis/core.kubernetes/ directly as an HTTPRoute backendRef allows this handler
// to validate the robot's JWT via token-vendor, replace the Authorization header
// in Go memory with the local ServiceAccount token, and stream requests directly to
// https://kubernetes.default.svc:443 without header collisions or Istio listener constraints.
func (h *handlers) proxyKubernetes(w http.ResponseWriter, r *http.Request) {
	encodedJWT := strings.TrimPrefix(r.Header.Get("Authorization"), "Bearer ")
	if err := h.verifyJWT(encodedJWT); err != nil {
		if *acceptLegacyCredentials && strings.HasPrefix(encodedJWT, legacyTokenPrefix) {
			// Allow legacy credentials through if configured.
		} else {
			http.Error(w, "No valid credentials provided", http.StatusUnauthorized)
			return
		}
	}

	var robotName string
	if strings.HasPrefix(encodedJWT, legacyTokenPrefix) {
		robotName = ""
	} else {
		claims, err := jws.Decode(encodedJWT)
		if err != nil {
			slog.Error("failed to parse JWT despite previous verification")
			http.Error(w, "Credentials could not be parsed", http.StatusInternalServerError)
			return
		}
		robotName = claims.Sub
		slog.Debug("JWT parsed", slog.String("id", robotName))
	}

	// For reverse proxy requests, validate r.URL.String() directly to guarantee 100% path alignment.
	if err := h.validateRequestPath(r.URL.String(), robotName); err != nil {
		http.Error(w, "Request not allowed", http.StatusForbidden)
		return
	}

	k8sToken, err := os.ReadFile(*k8sTokenPath)
	if err != nil {
		slog.Error("failed to read serviceaccount token", ilog.Err(err))
		http.Error(w, "Internal error", http.StatusInternalServerError)
		return
	}

	proxy := &httputil.ReverseProxy{
		Director: func(req *http.Request) {
			req.URL.Scheme = h.k8sTargetURL.Scheme
			req.URL.Host = h.k8sTargetURL.Host
			req.URL.Path = strings.TrimPrefix(req.URL.Path, "/apis/core.kubernetes")
			if req.URL.Path == "" {
				req.URL.Path = "/"
			}
			// Why: RawPath stores the original encoded path. Resetting RawPath = "" forces
			// httputil.ReverseProxy to re-encode req.URL.Path without retaining the stripped prefix.
			req.URL.RawPath = ""
			req.Header.Set("Authorization", "Bearer "+strings.TrimSpace(string(k8sToken)))
			req.Host = h.k8sTargetURL.Host
		},
		Transport:     h.k8sTransport,
		FlushInterval: -1, // Why: Flush immediately to prevent buffering on long-lived K8s API Watch streams.
	}
	proxy.ServeHTTP(w, r)
}

// auth is a webhook to inspect incoming requests from the cr-syncer, check if
// they are allowed, and if so, provide an Authorization header so the K8s
// apiserver will serve them. This lets nginx handle the request & response
// bodies itself.
func (h *handlers) auth(w http.ResponseWriter, r *http.Request) {
	encodedJWT := strings.TrimPrefix(r.Header.Get("Authorization"), "Bearer ")
	if err := h.verifyJWT(encodedJWT); err != nil {
		if *acceptLegacyCredentials {
			// The request already has the necessary credentials, so preserve these.
			w.Header().Add("Authorization", r.Header.Get("Authorization"))
			w.WriteHeader(http.StatusOK)
			return
		}

		http.Error(w, "No valid credentials provided", http.StatusUnauthorized)
		return
	}

	// verifyJWT() has already checked the signature so we don't need to.
	claims, err := jws.Decode(encodedJWT)
	if err != nil {
		slog.Error("Failed to parse JWT despite previous verification")
		http.Error(w, "Credentials could not be parsed", http.StatusInternalServerError)
		return
	}
	slog.Debug("JWT parsed", slog.String("id", claims.Sub))

	urlStr, err := extractSubrequestURL(r)
	if err != nil {
		slog.Error("failed to extract subrequest URL", ilog.Err(err))
		http.Error(w, "Invalid subrequest headers", http.StatusBadRequest)
		return
	}

	if err := h.validateRequestPath(urlStr, claims.Sub); err != nil {
		http.Error(w, "Request not allowed", http.StatusForbidden)
		return
	}

	// Provide a k8s token to nginx so that GKE accepts the request. Policy for
	// the cr-syncer-auth-webhook ServiceAccount is defined in
	// cr-syncer-policy.yaml.
	k8sToken, err := os.ReadFile(*k8sTokenPath)
	if err != nil {
		slog.Error("failed to read serviceaccount token", ilog.Err(err))
		http.Error(w, "Internal error", http.StatusInternalServerError)
		return
	}
	w.Header().Add("Authorization", "Bearer "+string(k8sToken))
	w.WriteHeader(http.StatusOK)
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
	http.HandleFunc("/apis/core.kubernetes/", handlers.proxyKubernetes)

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
