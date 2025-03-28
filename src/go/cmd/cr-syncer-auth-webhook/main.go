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
	"flag"
	"fmt"
	"io"
	"log/slog"
	"net/http"
	"os"
	"os/signal"
	"strings"
	"syscall"
	"time"

	"github.com/pkg/errors"
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
)

const (
	verifyJWTEndpoint = "/apis/core.token-vendor/v1/jwt.verify"

	legacyTokenPrefix = "ya29."
)

type handlers struct {
	client *http.Client
}

func newHandlers() handlers {
	return handlers{
		client: &http.Client{},
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
		return fmt.Errorf("legacy token format")
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
	// Discard body so connection can be reused.
	io.Copy(io.Discard, resp.Body)
	resp.Body.Close()

	if resp.StatusCode == http.StatusForbidden {
		return fmt.Errorf("invalid JWT")
	} else if resp.StatusCode != http.StatusOK {
		slog.Warn("unexpected status code from /jwt.verify", slog.Int("Status", resp.StatusCode))
		return fmt.Errorf("unexpected status code")
	}
	return nil
}

func (h *handlers) resourceIsFiltered(groupKind string) bool {
	// TODO: limit to CRDs with filter-by-robot-name label in case someone adds
	// new unfiltered resources in future.
	return groupKind != "registry.cloudrobotics.com/robottypes"
}

// validateRequest checks that the request is expected for the cr-syncer and
// only accesses allowed resources.
func (h *handlers) validateRequest(r *http.Request, robotName string) error {
	urlString := r.Header.Get("X-Original-Url")
	incomingReq, err := parseURL(urlString)
	if err != nil {
		slog.Error("unexpected value of X-Original-Url", slog.String("URL", urlString), ilog.Err(err))
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

	// TODO: check against label of upstream resource instead of assuming that
	// robot xyz can access all syncable resources matching *xyz.
	if incomingReq.RobotName != robotName && !strings.HasSuffix(incomingReq.ResourceName, robotName) {
		slog.Error("robot impersonation rejected",
			slog.String("SourceName", robotName),
			slog.String("TargetName", incomingReq.RobotName+incomingReq.ResourceName),
			slog.String("Kind", incomingReq.GroupKind),
			slog.String("URL", urlString),
		)
		return errors.New("credentials rejected")
	}
	return nil
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
	slog.Debug("JWT parsed", slog.String("ID", claims.Sub))

	if err := h.validateRequest(r, claims.Sub); err != nil {
		http.Error(w, "Request not allowed", http.StatusForbidden)
		return
	}

	// Provide a k8s token to nginx so that GKE accepts the request. Policy for
	// the cr-syncer-auth-webhook ServiceAccount is defined in
	// cr-syncer-policy.yaml.
	k8sToken, err := os.ReadFile("/var/run/secrets/kubernetes.io/serviceaccount/token")
	if err != nil {
		slog.Error("failed to read /var/run/secrets/kubernetes.io/serviceaccount/token", ilog.Err(err))
		http.Error(w, "Internal error", http.StatusInternalServerError)
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
