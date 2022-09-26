package tokensource

import (
	"testing"
	"time"

	"github.com/google/go-cmp/cmp"
	iam "google.golang.org/api/iamcredentials/v1"
)

type TokenResponseTest struct {
	desc    string
	req     iam.GenerateAccessTokenResponse
	scopes  []string
	now     time.Time
	tr      TokenResponse
	wantErr bool
}

func TestTokenResponse(t *testing.T) {
	now, _ := time.Parse(time.RFC3339Nano, "1986-06-30T15:01:23.045123456Z")
	var cases = []TokenResponseTest{
		{
			desc: "happy path",
			req: iam.GenerateAccessTokenResponse{
				AccessToken: "abc",
				ExpireTime:  "1986-06-30T15:02:06.045123456Z"},
			scopes:  []string{"a", "b"},
			now:     now,
			tr:      TokenResponse{AccessToken: "abc", ExpiresIn: 43, Scope: "a b", TokenType: "Bearer"},
			wantErr: false,
		},
	}

	for _, test := range cases {
		t.Run(test.desc, func(t *testing.T) {
			got, err := tokenResponse(&test.req, test.scopes, test.now)
			if (test.wantErr && err == nil) || (!test.wantErr) && err != nil {
				t.Fatalf("tokenResponse(..): got error %v, want %v", err, test.wantErr)
			}
			if diff := cmp.Diff(got, &test.tr); diff != "" {
				t.Fatalf("tokenResponse(..): got %+v, wanted %+v, diff %v", got, test.tr, diff)
			}
		})
	}
}
