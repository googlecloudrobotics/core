package tokensource

import (
	"context"
	"fmt"
	"log/slog"
	"math"
	"net/http"
	"strings"
	"sync/atomic"
	"time"

	"cloud.google.com/go/compute/metadata"
	"github.com/googlecloudrobotics/ilog"
	"github.com/pkg/errors"
	iam "google.golang.org/api/iamcredentials/v1"
	"google.golang.org/api/option"
)

type GCPTokenSource struct {
	service *iam.Service
	scopes  []string
}

type TokenResponse struct {
	AccessToken string `json:"access_token"`
	ExpiresIn   int64  `json:"expires_in"`
	Scope       string `json:"scope"`
	TokenType   string `json:"token_type"`
}

const (
	saPrefix = "projects/-/serviceAccounts/"
)

// NewGCPTokenSource creates a token source for GCP access tokens.
//
// `client` parameter is optional. If you supply your own client, you have to make
// sure you set the correct authentication headers yourself. If no client is given,
// authentication information is looked up from the environment.
// `defaultSAName` specifies the GCP IAM service accoutn name to use if no
// dedicated service account is configurred on the key.
func NewGCPTokenSource(ctx context.Context, client *http.Client, scopes []string) (*GCPTokenSource, error) {
	service, err := iam.NewService(ctx, option.WithHTTPClient(client))
	if err != nil {
		return nil, errors.Wrap(err, "failed to create IAM service client")
	}
	return &GCPTokenSource{service: service, scopes: scopes}, nil
}

// Token returns an access token for the configured service account and scopes.
//
// API: https://cloud.google.com/iam/docs/reference/credentials/rest/v1/projects.serviceAccounts/generateAccessToken
func (g *GCPTokenSource) Token(ctx context.Context, saName, saDelegateName string) (*TokenResponse, error) {
	if saName == "" {
		return nil, fmt.Errorf("saName must not be empty")
	}

	var delegates []string
	if saDelegateName != "" {
		// Impersonation was requested; constructing impersonation chain
		// [Pod's WI, saDelegateName]. For details on impersonation requirements
		// see: https://docs.cloud.google.com/iam/docs/service-account-impersonation
		if myself, err := getWorkloadServiceAccount(ctx); err == nil {
			delegates = append(delegates, saPrefix+myself)
		} else {
			// In general, impersonation without Pod's WI may not succeed, but we are going to proceed anyway, just in case.
			// That should also help with debugging of these issues as messages from IAM are fairly descriptive and allow
			// for better auditing then errors which we can produce here.
			slog.Warn("Token for delegate was requested but metadata server not available, ignoring and trying anyway", ilog.Err(err))
		}
		delegates = append(delegates, saPrefix+saDelegateName)
	}
	req := iam.GenerateAccessTokenRequest{
		Scope:     g.scopes,
		Delegates: delegates,
	}
	resource := saPrefix + saName
	slog.DebugContext(ctx, "Requesting token", slog.String("principal", resource), slog.Any("delegates", delegates))
	// We don't set a 'lifetime' on the request, so we get the default value (3600 sec = 1h).
	// This needs to be in sync with the min(cookie-expire,cookie-refresh) duration
	// configured on oauth2-proxy.
	resp, err := g.service.Projects.ServiceAccounts.
		GenerateAccessToken(resource, &req).Context(ctx).Do()
	if err != nil {
		return nil, errors.Wrapf(err, "GenerateAccessToken(..) for %q failed", resource)
	}
	tok, err := tokenResponse(resp, g.scopes, time.Now())
	if err != nil {
		return nil, errors.Wrapf(err, "failed to generate token response from GCP response")
	}
	return tok, nil
}

// tokenResponse returns a TokenResponse struct given an IAM response object.
func tokenResponse(r *iam.GenerateAccessTokenResponse, scopes []string, now time.Time) (*TokenResponse, error) {
	tr := TokenResponse{
		TokenType:   "Bearer",
		AccessToken: r.AccessToken,
		Scope:       strings.Join(scopes, " ")}
	// calculate ExpiresIn
	exp, err := time.Parse(time.RFC3339Nano, r.ExpireTime)
	if err != nil {
		return nil, errors.Wrapf(err, "failed to parse expiration time %q", r.ExpireTime)
	}
	diff := now.Sub(exp)
	tr.ExpiresIn = int64(math.Abs(diff.Seconds()))
	return &tr, nil
}

var workloadServiceAccount = new(atomic.Pointer[string])

// getWorkloadServiceAccount returns a service account email for the pod running
// token vendor if available. Value is cached, so subsequent calls save time
// on contacting metadata server.
func getWorkloadServiceAccount(ctx context.Context) (string, error) {
	if val := workloadServiceAccount.Load(); val != nil {
		return *val, nil
	}

	ctx, cancel := context.WithTimeout(ctx, 5*time.Second)
	defer cancel()

	// Fetch the email for the 'default' service account.
	// In Workload Identity, 'default' is the IAM SA mapped to the pod.
	email, err := metadata.EmailWithContext(ctx, "default")
	if err != nil {
		return "", fmt.Errorf("cannot obtain service account from metadata: %w", err)
	}
	// we are going to perform write ONLY if old value is nil, otherwise
	// we have the same value stored there anyway due to how metadata
	// server works for pods on GKE.
	workloadServiceAccount.CompareAndSwap(nil, &email)
	return email, nil
}
