package oauth

import (
	"context"
	"fmt"
	"net/http"
	"time"

	"github.com/pkg/errors"
	"google.golang.org/api/iam/v1"
	"google.golang.org/api/option"
)

type TokenVerifier struct {
	s       *iam.Service
	project string
	cache   *tokenCache
}

// Token as string alias empathizes that this is a secret.
type Token string

const (
	cacheSize   = 1000
	cacheExpire = 5 * time.Minute
)

// NewTokenVerifier returns a new TokenVerifier instance for a cloud project.
func NewTokenVerifier(ctx context.Context, c *http.Client, project string) (*TokenVerifier, error) {
	s, err := iam.NewService(ctx, option.WithHTTPClient(c))
	if err != nil {
		return nil, errors.Wrap(err, "failed to create service client")
	}
	tc, err := newTokenCache(cacheSize, cacheExpire)
	if err != nil {
		return nil, errors.Wrap(err, "failed to create token cache")
	}
	return &TokenVerifier{s: s, project: project, cache: tc}, nil
}

// Verify if a given token has "actAs" permission on a given service account.
func (v *TokenVerifier) Verify(ctx context.Context, token Token, sa string) error {
	resource := fmt.Sprintf("projects/%s/serviceAccounts/%s@%s.iam.gserviceaccount.com", v.project, sa, v.project)
	const iamActAs = "iam.serviceAccounts.actAs"
	// check the cache first
	actAs, found := v.cache.actAs(token, resource)
	if found && actAs {
		return nil
	}
	if found && !actAs {
		return fmt.Errorf("token is missing permission %q for resource %q (cached)",
			iamActAs, resource)
	}
	// query IAM if not found in cache
	preq := iam.TestIamPermissionsRequest{
		Permissions: []string{iamActAs},
	}
	pcall := v.s.Projects.ServiceAccounts.TestIamPermissions(resource, &preq)
	pcall.Header().Set("Authorization", "Bearer "+string(token))
	resp, err := pcall.Context(ctx).Do()
	if err != nil {
		return errors.Wrapf(err, "TestIamPermissions failed for resource %q with permission %q",
			resource, iamActAs)
	}
	if !contains(resp.Permissions, iamActAs) {
		v.cache.add(token, resource, false)
		return fmt.Errorf("token is missing permission %q for resource %q", iamActAs, resource)
	}
	v.cache.add(token, resource, true)
	return nil
}

func contains(s []string, str string) bool {
	for _, v := range s {
		if v == str {
			return true
		}
	}
	return false
}
