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
	"fmt"
	"log/slog"
	"net/url"
	"strings"

	authv3 "github.com/envoyproxy/go-control-plane/envoy/service/auth/v3"
	typev3 "github.com/envoyproxy/go-control-plane/envoy/type/v3"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/app"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/oauth"
	"github.com/googlecloudrobotics/ilog"
	"google.golang.org/genproto/googleapis/rpc/status"
	"google.golang.org/grpc/codes"
)

// ExtAuthzServer implements envoy.service.auth.v3.AuthorizationServer.
type ExtAuthzServer struct {
	authv3.UnimplementedAuthorizationServer
	tv *app.TokenVendor
}

// NewAuthorizationServer creates a new ExtAuthzServer with the given TokenVendor.
func NewAuthorizationServer(tv *app.TokenVendor) *ExtAuthzServer {
	return &ExtAuthzServer{tv: tv}
}

// Check handles Envoy ext_authz gRPC authorization requests.
func (s *ExtAuthzServer) Check(ctx context.Context, req *authv3.CheckRequest) (*authv3.CheckResponse, error) {
	if req == nil || req.GetAttributes() == nil {
		return deniedResponse(typev3.StatusCode_Unauthorized, codes.Unauthenticated, "missing request attributes"), nil
	}

	isRobotReq := isRobot(req)

	httpReq := req.GetAttributes().GetRequest().GetHttp()
	var headers map[string]string
	var pathStr string
	if httpReq != nil {
		headers = httpReq.GetHeaders()
		pathStr = httpReq.GetPath()
	}

	var endpoint string
	var extPath string
	if ext := req.GetAttributes().GetContextExtensions(); ext != nil {
		endpoint = strings.TrimSpace(ext["endpoint"])
		extPath = strings.TrimSpace(ext["path"])
	}

	// 1. Check X-Forwarded-Access-Token header
	tokenStr := ""
	if fwdToken := getHeader(headers, "x-forwarded-access-token"); fwdToken != "" {
		tokenStr = fwdToken
	} else if authHeader := getHeader(headers, "authorization"); authHeader != "" {
		tokenStr = strings.TrimSpace(authHeader)
		if strings.HasPrefix(strings.ToLower(tokenStr), "bearer ") {
			tokenStr = strings.TrimSpace(tokenStr[7:])
		}
	} else if pathStr != "" {
		if u, err := url.Parse(pathStr); err == nil {
			tokenStr = u.Query().Get("token")
		}
	}
	if tokenStr == "" && extPath != "" {
		if u, err := url.Parse(extPath); err == nil {
			tokenStr = u.Query().Get("token")
		}
	}

	if tokenStr == "" {
		return deniedResponse(typev3.StatusCode_Unauthorized, codes.Unauthenticated, "missing authorization credentials"), nil
	}

	if endpoint == "token.verify" || strings.Contains(endpoint, "token.verify") || strings.Contains(extPath, "/token.verify") || strings.Contains(pathStr, "/token.verify") {
		return s.verifyOAuthToken(ctx, tokenStr, isRobotReq)
	}

	if endpoint == "jwt.verify" || strings.Contains(endpoint, "jwt.verify") || strings.Contains(extPath, "/jwt.verify") || strings.Contains(pathStr, "/jwt.verify") {
		return s.verifyJWT(ctx, tokenStr)
	}

	return s.validateCredentials(ctx, tokenStr, isRobotReq)
}

func (s *ExtAuthzServer) verifyOAuthToken(ctx context.Context, tokenStr string, isRobot bool) (*authv3.CheckResponse, error) {
	if _, err := isValidToken(tokenStr); err != nil {
		return deniedResponse(typev3.StatusCode_BadRequest, codes.InvalidArgument, fmt.Sprintf("invalid access token format: %v", err)), nil
	}
	if err := s.tv.VerifyToken(ctx, oauth.Token(tokenStr), isRobot); err != nil {
		slog.WarnContext(ctx, "Token verification failed", ilog.Err(err))
		return deniedResponse(typev3.StatusCode_Forbidden, codes.PermissionDenied, "unable to verify token"), nil
	}
	return okResponse(), nil
}

func (s *ExtAuthzServer) verifyJWT(ctx context.Context, tokenStr string) (*authv3.CheckResponse, error) {
	if _, err := isValidJWT(tokenStr); err != nil {
		return deniedResponse(typev3.StatusCode_BadRequest, codes.InvalidArgument, fmt.Sprintf("invalid JWT format: %v", err)), nil
	}
	if _, err := s.tv.ValidateJWT(ctx, tokenStr); err != nil {
		slog.WarnContext(ctx, "JWT validation failed", ilog.Err(err))
		return deniedResponse(typev3.StatusCode_Forbidden, codes.PermissionDenied, fmt.Sprintf("JWT not valid: %v", err)), nil
	}
	return okResponse(), nil
}

func (s *ExtAuthzServer) validateCredentials(ctx context.Context, tokenStr string, isRobot bool) (*authv3.CheckResponse, error) {
	// Google OAuth access tokens start with ya29. and must be verified as OAuth tokens,
	// not JWTs (even if they contain dots matching loose regex).
	if strings.HasPrefix(tokenStr, "ya29.") {
		return s.verifyOAuthToken(ctx, tokenStr, isRobot)
	}

	if isRobot {
		if ok, _ := isValidJWT(tokenStr); ok {
			return s.verifyJWT(ctx, tokenStr)
		}
		if ok, _ := isValidToken(tokenStr); ok {
			return s.verifyOAuthToken(ctx, tokenStr, true)
		}
		return deniedResponse(typev3.StatusCode_BadRequest, codes.InvalidArgument, "invalid token format"), nil
	}

	// Human request
	if ok, _ := isValidToken(tokenStr); ok {
		return s.verifyOAuthToken(ctx, tokenStr, false)
	}

	return deniedResponse(typev3.StatusCode_BadRequest, codes.InvalidArgument, "invalid token format"), nil
}

func isRobot(req *authv3.CheckRequest) bool {
	if req.GetAttributes() == nil {
		return false
	}
	if ext := req.GetAttributes().GetContextExtensions(); ext != nil {
		if val, ok := ext["robot"]; ok {
			v := strings.ToLower(strings.TrimSpace(val))
			if v == "true" || v == "1" || v == "yes" {
				return true
			}
			if v == "false" || v == "0" || v == "no" {
				return false
			}
		}
		if val, ok := ext["robots"]; ok {
			v := strings.ToLower(strings.TrimSpace(val))
			if v == "true" || v == "1" || v == "yes" {
				return true
			}
			if v == "false" || v == "0" || v == "no" {
				return false
			}
		}
		if val, ok := ext["type"]; ok {
			v := strings.ToLower(strings.TrimSpace(val))
			if v == "robot" || v == "robots" {
				return true
			}
			if v == "human" || v == "user" {
				return false
			}
		}
		if scopes, ok := ext["scopes"]; ok {
			s := strings.ToLower(scopes)
			if strings.Contains(s, "robot") {
				return true
			}
		}
		if p, ok := ext["path"]; ok && p != "" {
			if u, err := url.Parse(p); err == nil {
				q := u.Query()
				if val := q.Get("robots"); val != "" {
					if strings.ToLower(strings.TrimSpace(val)) == "true" {
						return true
					}
					if strings.ToLower(strings.TrimSpace(val)) == "false" {
						return false
					}
				}
				if val := q.Get("robot"); val != "" {
					if strings.ToLower(strings.TrimSpace(val)) == "true" {
						return true
					}
					if strings.ToLower(strings.TrimSpace(val)) == "false" {
						return false
					}
				}
			}
		}
	}

	httpReq := req.GetAttributes().GetRequest().GetHttp()
	if httpReq != nil {
		for k, v := range httpReq.GetHeaders() {
			if strings.EqualFold(k, "x-crc-tv-robots") {
				v := strings.ToLower(strings.TrimSpace(v))
				if v == "true" || v == "1" || v == "yes" {
					return true
				}
				if v == "false" || v == "0" || v == "no" {
					return false
				}
			}
		}
		if pathStr := httpReq.GetPath(); pathStr != "" {
			if u, err := url.Parse(pathStr); err == nil {
				q := u.Query()
				if val := q.Get("robots"); val != "" {
					if strings.ToLower(strings.TrimSpace(val)) == "true" {
						return true
					}
					if strings.ToLower(strings.TrimSpace(val)) == "false" {
						return false
					}
				}
				if val := q.Get("robot"); val != "" {
					if strings.ToLower(strings.TrimSpace(val)) == "true" {
						return true
					}
					if strings.ToLower(strings.TrimSpace(val)) == "false" {
						return false
					}
				}
			}
		}
	}
	return false
}

func getHeader(headers map[string]string, key string) string {
	if headers == nil {
		return ""
	}
	if v, ok := headers[key]; ok {
		return v
	}
	for k, v := range headers {
		if strings.EqualFold(k, key) {
			return v
		}
	}
	return ""
}

func deniedResponse(httpCode typev3.StatusCode, rpcCode codes.Code, msg string) *authv3.CheckResponse {
	return &authv3.CheckResponse{
		Status: &status.Status{
			Code:    int32(rpcCode),
			Message: msg,
		},
		HttpResponse: &authv3.CheckResponse_DeniedResponse{
			DeniedResponse: &authv3.DeniedHttpResponse{
				Status: &typev3.HttpStatus{
					Code: httpCode,
				},
				Body: msg,
			},
		},
	}
}

func okResponse() *authv3.CheckResponse {
	return &authv3.CheckResponse{
		Status: &status.Status{
			Code: int32(codes.OK),
		},
		HttpResponse: &authv3.CheckResponse_OkResponse{
			OkResponse: &authv3.OkHttpResponse{},
		},
	}
}
