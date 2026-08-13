// Copyright 2026 The Cloud Robotics Authors
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

package v1

import (
	"context"
	"io"
	"net/http"
	"strings"
	"testing"

	authv3 "github.com/envoyproxy/go-control-plane/envoy/service/auth/v3"
	typev3 "github.com/envoyproxy/go-control-plane/envoy/type/v3"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/app"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/oauth"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/repository/k8s"
	"google.golang.org/grpc/codes"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/client-go/kubernetes/fake"
)

func setupTestExtAuthzServer(t *testing.T, iamHandler RoundTripFunc) *ExtAuthzServer {
	t.Helper()
	ctx := context.Background()

	cs := fake.NewSimpleClientset()
	if err := populateK8sEnv(ctx, cs, "default",
		[]*corev1.ConfigMap{
			{
				TypeMeta: metav1.TypeMeta{
					Kind:       "ConfigMap",
					APIVersion: "v1",
				},
				ObjectMeta: metav1.ObjectMeta{
					Name: "robot-dev-testuser",
				},
				Data: map[string]string{"pubKey": "-----BEGIN PUBLIC KEY-----\nMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAvTGUksynbWhvZkHNJn8C2oXVD400jiK4T0JoyS/SwbBGwFr3OJGlPwXCsvAPAzmpTuZpge6T3pnIcO/s97sMgyld9ZYio7SQiiRV/nwYZittGf9/yfHSNDJUvT25yhuK2p3UqRCom1a3KljeXbxXvGuYG48IH0kqAQbYBI/0lAV3H5pkdXPFZC6PHltC3jySVIOg7qPXrNuxdxmg/gmzQ9+NmKvXWKATAPax1yYoESaZtc22aCZWouIdJr3baYlfBb4w8stoJPoONuyn4ard17gywb46HHGl2XoY+Y5pihwvctsFeZXLfYwUmFPfgncQHJ02lCV3+Xyk4AAZy3xDpwIDAQAB\n-----END PUBLIC KEY-----"},
			},
		}); err != nil {
		t.Fatalf("failed to populate k8s env: %v", err)
	}

	repo, err := k8s.NewK8sRepository(ctx, cs, "default")
	if err != nil {
		t.Fatalf("failed to create k8s repo: %v", err)
	}

	var httpClient *http.Client
	if iamHandler != nil {
		httpClient = NewTestHTTPClient(iamHandler)
	} else {
		httpClient = NewTestHTTPClient(func(req *http.Request) *http.Response {
			return &http.Response{
				StatusCode: http.StatusOK,
				Body:       io.NopCloser(strings.NewReader(`{"permissions":["iam.serviceAccounts.actAs"]}`)),
				Header:     make(http.Header),
			}
		})
	}

	verifier, err := oauth.NewTokenVerifier(ctx, httpClient, "testproject")
	if err != nil {
		t.Fatalf("failed to create verifier: %v", err)
	}

	tv, err := app.NewTokenVendor(ctx, repo, verifier, nil, "testaud", saName)
	if err != nil {
		t.Fatalf("failed to create token vendor: %v", err)
	}

	return NewAuthorizationServer(tv)
}

func TestExtAuthzCheck_NilRequest(t *testing.T) {
	srv := setupTestExtAuthzServer(t, nil)

	resp, err := srv.Check(context.Background(), nil)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if resp.GetStatus().GetCode() != int32(codes.Unauthenticated) {
		t.Errorf("got rpc code %v, want %v", resp.GetStatus().GetCode(), codes.Unauthenticated)
	}
	denied := resp.GetDeniedResponse()
	if denied == nil {
		t.Fatalf("expected DeniedResponse, got nil")
	}
	if denied.GetStatus().GetCode() != typev3.StatusCode_Unauthorized {
		t.Errorf("got http code %v, want %v", denied.GetStatus().GetCode(), typev3.StatusCode_Unauthorized)
	}
}

func TestExtAuthzCheck_MissingCredentials(t *testing.T) {
	srv := setupTestExtAuthzServer(t, nil)

	req := &authv3.CheckRequest{
		Attributes: &authv3.AttributeContext{
			Request: &authv3.AttributeContext_Request{
				Http: &authv3.AttributeContext_HttpRequest{
					Method:  "GET",
					Path:    "/test",
					Headers: map[string]string{},
				},
			},
		},
	}

	resp, err := srv.Check(context.Background(), req)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	denied := resp.GetDeniedResponse()
	if denied == nil {
		t.Fatalf("expected DeniedResponse, got nil")
	}
	if denied.GetStatus().GetCode() != typev3.StatusCode_Unauthorized {
		t.Errorf("got http code %v, want %v", denied.GetStatus().GetCode(), typev3.StatusCode_Unauthorized)
	}
}

func TestExtAuthzCheck_RobotJWT(t *testing.T) {
	srv := setupTestExtAuthzServer(t, nil)

	tests := []struct {
		name         string
		contextExt   map[string]string
		headers      map[string]string
		path         string
		wantOK       bool
		wantHTTPCode typev3.StatusCode
		wantRPCCode  codes.Code
	}{
		{
			name: "valid robot jwt with contextExtension robot=true",
			contextExt: map[string]string{
				"robot": "true",
			},
			headers: map[string]string{
				"authorization": "Bearer " + jwtCorrect,
			},
			wantOK: true,
		},

		{
			name: "valid robot jwt with header x-crc-tv-robots=true",
			headers: map[string]string{
				"x-crc-tv-robots": "true",
				"authorization":   "Bearer " + jwtCorrect,
			},
			wantOK: true,
		},
		{
			name: "valid robot jwt with path query robots=true",
			path: "/test?robots=true",
			headers: map[string]string{
				"authorization": "Bearer " + jwtCorrect,
			},
			wantOK: true,
		},
		{
			name: "robot jwt with wrong signature",
			contextExt: map[string]string{
				"robot": "true",
			},
			headers: map[string]string{
				"authorization": "Bearer " + jwtWrongSig,
			},
			wantOK:       false,
			wantHTTPCode: typev3.StatusCode_Forbidden,
			wantRPCCode:  codes.PermissionDenied,
		},
		{
			name: "robot jwt with wrong audience",
			contextExt: map[string]string{
				"robot": "true",
			},
			headers: map[string]string{
				"authorization": "Bearer " + jwtWrongAud,
			},
			wantOK:       false,
			wantHTTPCode: typev3.StatusCode_Forbidden,
			wantRPCCode:  codes.PermissionDenied,
		},
		{
			name: "robot jwt expired",
			contextExt: map[string]string{
				"robot": "true",
			},
			headers: map[string]string{
				"authorization": "Bearer " + jwtExpired,
			},
			wantOK:       false,
			wantHTTPCode: typev3.StatusCode_Forbidden,
			wantRPCCode:  codes.PermissionDenied,
		},
	}

	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			path := tc.path
			if path == "" {
				path = "/apis/core.kubernetes-relay/server"
			}
			req := &authv3.CheckRequest{
				Attributes: &authv3.AttributeContext{
					ContextExtensions: tc.contextExt,
					Request: &authv3.AttributeContext_Request{
						Http: &authv3.AttributeContext_HttpRequest{
							Method:  "GET",
							Path:    path,
							Headers: tc.headers,
						},
					},
				},
			}

			resp, err := srv.Check(context.Background(), req)
			if err != nil {
				t.Fatalf("unexpected error: %v", err)
			}

			if tc.wantOK {
				if resp.GetStatus().GetCode() != int32(codes.OK) {
					t.Errorf("got rpc code %v, want %v", resp.GetStatus().GetCode(), codes.OK)
				}
				if resp.GetOkResponse() == nil {
					t.Fatalf("expected OkResponse, got nil")
				}
			} else {
				if resp.GetStatus().GetCode() != int32(tc.wantRPCCode) {
					t.Errorf("got rpc code %v, want %v", resp.GetStatus().GetCode(), tc.wantRPCCode)
				}
				denied := resp.GetDeniedResponse()
				if denied == nil {
					t.Fatalf("expected DeniedResponse, got nil")
				}
				if denied.GetStatus().GetCode() != tc.wantHTTPCode {
					t.Errorf("got http code %v, want %v", denied.GetStatus().GetCode(), tc.wantHTTPCode)
				}
			}
		})
	}
}

func TestExtAuthzCheck_OAuth2AccessToken(t *testing.T) {
	validToken := "ya29." + strings.Repeat("a", 100)

	tests := []struct {
		name         string
		contextExt   map[string]string
		headers      map[string]string
		path         string
		iamHasPerm   bool
		iamStatus    int
		wantOK       bool
		wantHTTPCode typev3.StatusCode
		wantRPCCode  codes.Code
	}{
		{
			name: "human token via authorization header happy path",
			headers: map[string]string{
				"authorization": "Bearer " + validToken,
			},
			iamHasPerm: true,
			iamStatus:  http.StatusOK,
			wantOK:     true,
		},
		{
			name: "human token via x-forwarded-access-token happy path",
			headers: map[string]string{
				"x-forwarded-access-token": validToken,
			},
			iamHasPerm: true,
			iamStatus:  http.StatusOK,
			wantOK:     true,
		},
		{
			name: "robot token via authorization header with contextExtension robot=true",
			contextExt: map[string]string{
				"robot": "true",
			},
			headers: map[string]string{
				"authorization": "Bearer " + validToken,
			},
			iamHasPerm: true,
			iamStatus:  http.StatusOK,
			wantOK:     true,
		},
		{
			name: "robot token with multiple dots (ya29.c.c0AY...) via authorization header",
			contextExt: map[string]string{
				"robot": "true",
			},
			headers: map[string]string{
				"authorization": "Bearer ya29.c." + strings.Repeat("a", 100),
			},
			iamHasPerm: true,
			iamStatus:  http.StatusOK,
			wantOK:     true,
		},

		{
			name:       "token via query parameter happy path",
			path:       "/test?token=" + validToken,
			headers:    map[string]string{},
			iamHasPerm: true,
			iamStatus:  http.StatusOK,
			wantOK:     true,
		},
		{
			name: "human token missing IAM permission",
			headers: map[string]string{
				"authorization": "Bearer " + validToken,
			},
			iamHasPerm:   false,
			iamStatus:    http.StatusOK,
			wantOK:       false,
			wantHTTPCode: typev3.StatusCode_Forbidden,
			wantRPCCode:  codes.PermissionDenied,
		},
		{
			name: "IAM API returned error",
			headers: map[string]string{
				"authorization": "Bearer " + validToken,
			},
			iamHasPerm:   true,
			iamStatus:    http.StatusInternalServerError,
			wantOK:       false,
			wantHTTPCode: typev3.StatusCode_Forbidden,
			wantRPCCode:  codes.PermissionDenied,
		},
		{
			name: "invalid token format",
			headers: map[string]string{
				"authorization": "Bearer invalid_short_token",
			},
			wantOK:       false,
			wantHTTPCode: typev3.StatusCode_BadRequest,
			wantRPCCode:  codes.InvalidArgument,
		},
	}

	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			iamHandler := func(req *http.Request) *http.Response {
				if tc.iamStatus != http.StatusOK {
					return &http.Response{
						StatusCode: tc.iamStatus,
						Body:       io.NopCloser(strings.NewReader("internal error")),
						Header:     make(http.Header),
					}
				}
				body := `{"permissions":[]}`
				if tc.iamHasPerm {
					body = `{"permissions":["iam.serviceAccounts.actAs"]}`
				}
				return &http.Response{
					StatusCode: http.StatusOK,
					Body:       io.NopCloser(strings.NewReader(body)),
					Header:     make(http.Header),
				}
			}

			srv := setupTestExtAuthzServer(t, iamHandler)

			path := tc.path
			if path == "" {
				path = "/apis/core.kubernetes-relay/client"
			}
			req := &authv3.CheckRequest{
				Attributes: &authv3.AttributeContext{
					ContextExtensions: tc.contextExt,
					Request: &authv3.AttributeContext_Request{
						Http: &authv3.AttributeContext_HttpRequest{
							Method:  "GET",
							Path:    path,
							Headers: tc.headers,
						},
					},
				},
			}

			resp, err := srv.Check(context.Background(), req)
			if err != nil {
				t.Fatalf("unexpected error: %v", err)
			}

			if tc.wantOK {
				if resp.GetStatus().GetCode() != int32(codes.OK) {
					t.Errorf("got rpc code %v, want %v", resp.GetStatus().GetCode(), codes.OK)
				}
				if resp.GetOkResponse() == nil {
					t.Fatalf("expected OkResponse, got nil")
				}
			} else {
				if resp.GetStatus().GetCode() != int32(tc.wantRPCCode) {
					t.Errorf("got rpc code %v, want %v", resp.GetStatus().GetCode(), tc.wantRPCCode)
				}
				denied := resp.GetDeniedResponse()
				if denied == nil {
					t.Fatalf("expected DeniedResponse, got nil")
				}
				if denied.GetStatus().GetCode() != tc.wantHTTPCode {
					t.Errorf("got http code %v, want %v", denied.GetStatus().GetCode(), tc.wantHTTPCode)
				}
			}
		})
	}
}

func TestExtAuthzCheck_EndpointDifferentiated(t *testing.T) {
	validOAuthToken := "ya29." + strings.Repeat("a", 100)

	tests := []struct {
		name         string
		contextExt   map[string]string
		headers      map[string]string
		path         string
		wantOK       bool
		wantHTTPCode typev3.StatusCode
		wantRPCCode  codes.Code
	}{
		{
			name: "endpoint token.verify with valid oauth token and robot=false",
			contextExt: map[string]string{
				"endpoint": "token.verify",
				"robot":    "false",
			},
			headers: map[string]string{
				"authorization": "Bearer " + validOAuthToken,
			},
			wantOK: true,
		},
		{
			name: "endpoint token.verify with valid oauth token and robot=true",
			contextExt: map[string]string{
				"endpoint": "token.verify",
				"robot":    "true",
			},
			headers: map[string]string{
				"authorization": "Bearer " + validOAuthToken,
			},
			wantOK: true,
		},
		{
			name: "endpoint jwt.verify with valid jwt",
			contextExt: map[string]string{
				"endpoint": "jwt.verify",
			},
			headers: map[string]string{
				"authorization": "Bearer " + jwtCorrect,
			},
			wantOK: true,
		},
		{
			name: "path /apis/core.token-vendor/v1/token.verify?robots=true with valid oauth token",
			path: "/apis/core.token-vendor/v1/token.verify?robots=true",
			headers: map[string]string{
				"authorization": "Bearer " + validOAuthToken,
			},
			wantOK: true,
		},
		{
			name: "path /apis/core.token-vendor/v1/jwt.verify with valid jwt",
			path: "/apis/core.token-vendor/v1/jwt.verify",
			headers: map[string]string{
				"authorization": "Bearer " + jwtCorrect,
			},
			wantOK: true,
		},
		{
			name: "contextExtensions path /apis/core.token-vendor/v1/token.verify?robots=true",
			contextExt: map[string]string{
				"path": "/apis/core.token-vendor/v1/token.verify?robots=true",
			},
			headers: map[string]string{
				"authorization": "Bearer " + validOAuthToken,
			},
			wantOK: true,
		},
		{
			name: "contextExtensions path /apis/core.token-vendor/v1/jwt.verify",
			contextExt: map[string]string{
				"path": "/apis/core.token-vendor/v1/jwt.verify",
			},
			headers: map[string]string{
				"authorization": "Bearer " + jwtCorrect,
			},
			wantOK: true,
		},
		{
			name: "endpoint token.verify rejects non-oauth format token",
			contextExt: map[string]string{
				"endpoint": "token.verify",
			},
			headers: map[string]string{
				"authorization": "Bearer " + jwtCorrect,
			},
			wantOK:       false,
			wantHTTPCode: typev3.StatusCode_BadRequest,
			wantRPCCode:  codes.InvalidArgument,
		},
		{
			name: "endpoint jwt.verify rejects oauth token format",
			contextExt: map[string]string{
				"endpoint": "jwt.verify",
			},
			headers: map[string]string{
				"authorization": "Bearer " + validOAuthToken,
			},
			wantOK:       false,
			wantHTTPCode: typev3.StatusCode_BadRequest,
			wantRPCCode:  codes.InvalidArgument,
		},
	}

	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			iamHandler := func(req *http.Request) *http.Response {
				body := `{"permissions":["iam.serviceAccounts.actAs"]}`
				return &http.Response{
					StatusCode: http.StatusOK,
					Body:       io.NopCloser(strings.NewReader(body)),
					Header:     make(http.Header),
				}
			}

			srv := setupTestExtAuthzServer(t, iamHandler)

			path := tc.path
			if path == "" {
				path = "/apis/core.kubernetes-relay/client"
			}
			req := &authv3.CheckRequest{
				Attributes: &authv3.AttributeContext{
					ContextExtensions: tc.contextExt,
					Request: &authv3.AttributeContext_Request{
						Http: &authv3.AttributeContext_HttpRequest{
							Method:  "GET",
							Path:    path,
							Headers: tc.headers,
						},
					},
				},
			}

			resp, err := srv.Check(context.Background(), req)
			if err != nil {
				t.Fatalf("unexpected error: %v", err)
			}

			if tc.wantOK {
				if resp.GetStatus().GetCode() != int32(codes.OK) {
					t.Errorf("got rpc code %v, want %v", resp.GetStatus().GetCode(), codes.OK)
				}
				if resp.GetOkResponse() == nil {
					t.Fatalf("expected OkResponse, got nil")
				}
			} else {
				if resp.GetStatus().GetCode() != int32(tc.wantRPCCode) {
					t.Errorf("got rpc code %v, want %v", resp.GetStatus().GetCode(), tc.wantRPCCode)
				}
				denied := resp.GetDeniedResponse()
				if denied == nil {
					t.Fatalf("expected DeniedResponse, got nil")
				}
				if denied.GetStatus().GetCode() != tc.wantHTTPCode {
					t.Errorf("got http code %v, want %v", denied.GetStatus().GetCode(), tc.wantHTTPCode)
				}
			}
		})
	}
}
