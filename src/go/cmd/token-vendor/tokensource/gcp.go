package tokensource

import (
	"context"
	"fmt"
	"math"
	"net/http"
	"strings"
	"time"

	"github.com/pkg/errors"
	iam "google.golang.org/api/iamcredentials/v1"
	"google.golang.org/api/option"
)

type GCPTokenSource struct {
	service *iam.Service
	// the FQN of the service account
	resource string
	scopes   []string
}

type TokenResponse struct {
	AccessToken string `json:"access_token"`
	ExpiresIn   int64  `json:"expires_in"`
	Scope       string `json:"scope"`
	TokenType   string `json:"token_type"`
}

// NewGCPTokenSource creates a token source for GCP access tokens.
//
// `client` parameter is optional. If you supply your own client, you have to make
// sure you set the correct authentication headers yourself. If no client is given,
// authentication information is looked up from the environment.
func NewGCPTokenSource(ctx context.Context, client *http.Client, project, sa string, scopes []string) (*GCPTokenSource, error) {
	service, err := iam.NewService(ctx, option.WithHTTPClient(client))
	if err != nil {
		return nil, errors.Wrap(err, "failed to create IAM service client")
	}
	resource := fmt.Sprintf("projects/-/serviceAccounts/%s@%s.iam.gserviceaccount.com", sa, project)
	return &GCPTokenSource{service: service, resource: resource, scopes: scopes}, nil
}

// Token returns an access token for the configured service account and scopes.
//
// API: https://cloud.google.com/iam/docs/reference/credentials/rest/v1/projects.serviceAccounts/generateAccessToken
func (g *GCPTokenSource) Token(ctx context.Context) (*TokenResponse, error) {
	req := iam.GenerateAccessTokenRequest{Scope: g.scopes}
	// We don't set a 'lifetime' on the request, so we get the default value (3600 sec = 1h).
	// This needs to be in sync with the min(cookie-expire,cookie-refresh) duration
	// configured on oauth2-proxy.
	resp, err := g.service.Projects.ServiceAccounts.
		GenerateAccessToken(g.resource, &req).Context(ctx).Do()
	if err != nil {
		return nil, errors.Wrapf(err, "GenerateAccessToken(..) for %q failed", g.resource)
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
