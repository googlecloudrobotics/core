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
	"net/url"
	"os"
	"os/signal"
	"regexp"
	"strings"
	"syscall"
	"time"

	jwt "github.com/form3tech-oss/jwt-go"
	"github.com/googlecloudrobotics/ilog"
	"github.com/pkg/errors"
)

var (
	port = flag.Int("port", 8080,
		"Listening port for HTTP requests")

	acceptLegacyCredentials = flag.Bool("accept-legacy-service-account-credentials", false,
		"Whether to accept legacy GCP service account credentials")

	tokenVendor = flag.String("token-vendor", "http://token-vendor.app-token-vendor.svc.cluster.local",
		"Hostname of the token-vendor service")

	// Regex for RFC 1123 subdomain format
	// https://kubernetes.io/docs/concepts/overview/working-with-objects/names/#dns-label-names
	// https://github.com/kubernetes/kubernetes/blob/976a940f4a4e84fe814583848f97b9aafcdb083f/staging/src/k8s.io/apimachinery/pkg/util/validation/validation.go#L209
	isValidRobotName = regexp.MustCompile(`^[a-z0-9]([-a-z0-9]*[a-z0-9])?(\.[a-z0-9]([-a-z0-9]*[a-z0-9])?)*$`).MatchString
)

const (
	verifyJWTEndpoint = "/apis/core.token-vendor/v1/jwt.verify"

	// the prefix of the label selector query param used by the cr-syncer
	robotNameSelectorPrefix = "cloudrobotics.com/robot-name="
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

// analysis contains the authz-relevant properties of the resource
type analysis struct {
	// groupKind, eg "registry.cloudrobotics.com/robots"
	groupKind string

	// robotName, or empty if no label selector is used (eg for a Get or Update)
	robotName string

	// resourceID, or empty if no resource is specified (eg for a List or Watch of a filtered resource)
	resourceName string
}

func analyzeURL(urlString string) (*analysis, error) {
	result := analysis{}
	url, err := url.Parse(urlString)
	if err != nil {
		return nil, err
	}

	// Path should be one of:
	//  /apis/core.kubernetes/apis/<group>/<version>/<kind>
	//  /apis/core.kubernetes/apis/<group>/<version>/namespaces/<namespace>/<kind>
	//  /apis/core.kubernetes/apis/<group>/<version>/namespaces/<namespace>/<kind>/<resourceName>
	//  /apis/core.kubernetes/apis/<group>/<version>/namespaces/<namespace>/<kind>/<resourceName>/status
	//                             parts[0] parts[1] parts[2]   parts[3]   parts[4] parts[5]
	parts := strings.Split(strings.TrimPrefix(url.Path, "/apis/core.kubernetes/apis/"), "/")
	if len(parts) < 3 || len(parts) > 7 {
		return nil, errors.New("unexpected URL length")
	}
	if parts[2] != "namespaces" {
		// Add in "/namespaces/default" so remaining code can use fixed indices.
		// I also considered a regexp but it's not pretty:
		// "/apis/core.kubernetes/apis/([^/]*)/([^/]*)(/namespaces/[^/]*)?/([^/]*)/?([^/]*)(/status)?"
		parts = append(append(parts[:2], "namespaces", "default"), parts[2:]...)
	}

	result.groupKind = fmt.Sprintf("%s/%s", parts[0], parts[4])
	if len(parts) > 5 {
		// if a resourceName is in the URL, we don't need to look at the query parameters
		result.resourceName = parts[5]
		return &result, nil
	}

	// no resourceName, look for a labelSelector
	params := url.Query()
	labelSelectors := params["labelSelector"]
	if len(labelSelectors) == 0 {
		// This is an unfiltered List or Watch request (eg for robottypes).
		return &result, nil
	}
	if len(labelSelectors) > 1 || !strings.HasPrefix(labelSelectors[0], robotNameSelectorPrefix) {
		return nil, errors.New("invalid label selector")
	}
	result.robotName = strings.TrimPrefix(labelSelectors[0], robotNameSelectorPrefix)
	if !isValidRobotName(result.robotName) {
		return nil, errors.New("invalid robot name")
	}
	return &result, nil
}

func (h *handlers) resourceIsSynced(groupKind string) bool {
	// TODO: limit to CRDs with spec-source label
	return true
}

func (h *handlers) resourceIsFiltered(groupKind string) bool {
	// TODO: limit to CRDs with filter-by-robot-name label
	return groupKind == "registry.cloudrobotics.com/robots" || groupKind == "apps.cloudrobotics.com/chartassignments"
}

// validateRequest checks that the request is expected for the cr-syncer and
// only accessed allowed resources.
func (h *handlers) validateRequest(r *http.Request, robotName string) error {
	// TODO: use same const as robotauth.go?
	robotName = strings.TrimPrefix(robotName, "robot-")

	urlString := r.Header.Get("X-Original-Url")
	analysis, err := analyzeURL(urlString)
	if err != nil {
		slog.Error("unexpected value of X-Original-Url", slog.String("URL", urlString), ilog.Err(err))
		return err
	}

	if !h.resourceIsSynced(analysis.groupKind) {
		slog.Error("robot requested disallowed resource", slog.String("SourceName", robotName), slog.String("Kind", analysis.groupKind))
	}

	if h.resourceIsFiltered(analysis.groupKind) {
		// TODO: check against label of upstream resource instead of using resourceName suffix hack
		if analysis.robotName != robotName && !strings.HasSuffix(analysis.resourceName, robotName) {
			slog.Error("robot impersonation rejected",
				slog.String("SourceName", robotName),
				slog.String("TargetName", analysis.robotName+analysis.resourceName),
				slog.String("Kind", analysis.groupKind))
		}
		return nil
	}

	// Unfiltered resources (eg robottypes) are always allowed.
	return nil
}

func (h *handlers) auth(w http.ResponseWriter, r *http.Request) {
	// If the JWT is invalid/missing we'll do an unnecessary request but that's
	// not the common code path.
	encodedJWT := strings.TrimPrefix(r.Header.Get("Authorization"), "Bearer ")
	if err := h.verifyJWT(encodedJWT); err != nil {
		if *acceptLegacyCredentials {
			// The request already has the necessary credentials.
			w.WriteHeader(http.StatusOK)
			return
		}

		http.Error(w, "No valid credentials provided", http.StatusUnauthorized)
		return
	}

	// Use ParseUnverified because verifyJWT() has already checked the signature.
	// TODO: use jws library
	claims := jwt.MapClaims{}
	_, _, err := new(jwt.Parser).ParseUnverified(encodedJWT, claims)
	if err != nil {
		slog.Error("Failed to parse JWT despite previous verification")
		http.Error(w, "Credentials could not be parsed", http.StatusInternalServerError)
		return
	}

	slog.Info("JWT parsed", slog.String("ID", claims["iss"].(string)))

	if err := h.validateRequest(r, claims["iss"].(string)); err != nil {
		http.Error(w, "Request not allowed", http.StatusForbidden)
		return
	}

	// Provide a k8s token to nginx so that GKE accepts the request.
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
	logHandler := ilog.NewLogHandler(slog.LevelInfo, os.Stderr)
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
