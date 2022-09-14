package oauth

import (
	"context"
	"fmt"
	"net/http"

	"github.com/pkg/errors"
	"google.golang.org/api/iam/v1"
	"google.golang.org/api/option"
)

type TokenVerifier struct {
	s       *iam.Service
	project string
}

// Token as string alias empathizes that this is a secret.
type Token string

// NewTokenVerifier returns a new TokenVerifier instance for a cloud project.
func NewTokenVerifier(ctx context.Context, c *http.Client, project string) (*TokenVerifier, error) {
	s, err := iam.NewService(ctx, option.WithHTTPClient(c))
	if err != nil {
		return nil, errors.Wrap(err, "failed to create service client")
	}
	return &TokenVerifier{s: s, project: project}, nil
}

// Verify if a given token has "actAs" permission on a given service account.
func (v *TokenVerifier) Verify(ctx context.Context, token Token, sa string) error {
	saFQN := fmt.Sprintf("%s@%s.iam.gserviceaccount.com", sa, v.project)
	resource := fmt.Sprintf("projects/%s/serviceAccounts/%s", v.project, saFQN)
	const actAs = "iam.serviceAccounts.actAs"
	preq := iam.TestIamPermissionsRequest{
		Permissions: []string{actAs},
	}
	pcall := v.s.Projects.ServiceAccounts.TestIamPermissions(resource, &preq)
	pcall.Header().Set("Authorization", "Bearer "+string(token))
	resp, err := pcall.Context(ctx).Do()
	if err != nil {
		return errors.Wrapf(err, "TestIamPermissions failed for resource %q with permission %q",
			resource, actAs)
	}
	if !contains(resp.Permissions, actAs) {
		return fmt.Errorf("token is missing permission %q for resource %q", actAs, resource)
	}
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
