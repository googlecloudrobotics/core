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

// NewExtAuthzServer creates a new ExtAuthzServer with the given TokenVendor.
func NewExtAuthzServer(tv *app.TokenVendor) *ExtAuthzServer {
	return &ExtAuthzServer{tv: tv}
}

// Check handles Envoy ext_authz gRPC authorization requests.
func (s *ExtAuthzServer) Check(ctx context.Context, req *authv3.CheckRequest) (*authv3.CheckResponse, error) {
	if req == nil || req.GetAttributes() == nil {
		return deniedResponse(typev3.StatusCode_Unauthorized, codes.Unauthenticated, "missing request attributes"), nil
	}

	isRobotReq := isRobot(req)

	var headers map[string]string
	var reqPath string
	if httpReq := req.GetAttributes().GetRequest().GetHttp(); httpReq != nil {
		headers = httpReq.GetHeaders()
		reqPath = httpReq.GetPath()
	}

	var endpoint string
	var extPath string
	if ext := req.GetAttributes().GetContextExtensions(); ext != nil {
		endpoint = strings.TrimSpace(ext["endpoint"])
		extPath = strings.TrimSpace(ext["path"])
	}

	// 1. Check X-Forwarded-Access-Token header
	token := ""
	if fwdToken := getHeader(headers, "x-forwarded-access-token"); fwdToken != "" {
		token = fwdToken
	} else if authHeader := getHeader(headers, "authorization"); authHeader != "" {
		token = strings.TrimSpace(authHeader)
		const bearerPrefix = "bearer "
		if strings.HasPrefix(strings.ToLower(token), bearerPrefix) {
			token = strings.TrimSpace(token[len(bearerPrefix):])
		}
	} else if reqPath != "" && strings.Contains(reqPath, "?") {
		if u, err := url.Parse(reqPath); err == nil {
			token = u.Query().Get("token")
		}
	}
	if token == "" && extPath != "" && strings.Contains(extPath, "?") {
		if u, err := url.Parse(extPath); err == nil {
			token = u.Query().Get("token")
		}
	}

	if token == "" {
		return deniedResponse(typev3.StatusCode_Unauthorized, codes.Unauthenticated, "missing authorization credentials"), nil
	}

	if strings.HasPrefix(endpoint, "token.verify") || hasPathSuffix(extPath, "/token.verify") || hasPathSuffix(reqPath, "/token.verify") {
		return s.verifyOAuthToken(ctx, token, isRobotReq)
	}

	if strings.HasPrefix(endpoint, "jwt.verify") || hasPathSuffix(extPath, "/jwt.verify") || hasPathSuffix(reqPath, "/jwt.verify") {
		return s.verifyJWT(ctx, token)
	}

	return s.validateCredentials(ctx, token, isRobotReq)
}

func hasPathSuffix(rawURL, suffix string) bool {
	if rawURL == "" {
		return false
	}
	u, err := url.Parse(rawURL)
	if err != nil {
		return strings.HasSuffix(rawURL, suffix)
	}
	return strings.HasSuffix(u.Path, suffix) || u.Path == strings.TrimPrefix(suffix, "/")
}

func (s *ExtAuthzServer) verifyOAuthTokenInternal(ctx context.Context, token string, isRobot bool) (*authv3.CheckResponse, error) {
	if _, err := isValidToken(token); err != nil {
		return nil, err
	}
	if err := s.tv.VerifyToken(ctx, oauth.Token(token), isRobot); err != nil {
		slog.WarnContext(ctx, "Token verification failed", ilog.Err(err))
		return deniedResponse(typev3.StatusCode_Forbidden, codes.PermissionDenied, "unable to verify token"), nil
	}
	return okResponse(), nil
}

func (s *ExtAuthzServer) verifyOAuthToken(ctx context.Context, token string, isRobot bool) (*authv3.CheckResponse, error) {
	resp, err := s.verifyOAuthTokenInternal(ctx, token, isRobot)
	if err != nil {
		return deniedResponse(typev3.StatusCode_BadRequest, codes.InvalidArgument, fmt.Sprintf("invalid access token format: %v", err)), nil
	}
	return resp, nil
}

func (s *ExtAuthzServer) verifyJWTInternal(ctx context.Context, token string) (*authv3.CheckResponse, error) {
	if _, err := isValidJWT(token); err != nil {
		return nil, err
	}
	if _, err := s.tv.ValidateJWT(ctx, token); err != nil {
		slog.WarnContext(ctx, "JWT validation failed", ilog.Err(err))
		return deniedResponse(typev3.StatusCode_Forbidden, codes.PermissionDenied, fmt.Sprintf("JWT not valid: %v", err)), nil
	}
	return okResponse(), nil
}

func (s *ExtAuthzServer) verifyJWT(ctx context.Context, token string) (*authv3.CheckResponse, error) {
	resp, err := s.verifyJWTInternal(ctx, token)
	if err != nil {
		return deniedResponse(typev3.StatusCode_BadRequest, codes.InvalidArgument, fmt.Sprintf("invalid JWT format: %v", err)), nil
	}
	return resp, nil
}

func (s *ExtAuthzServer) validateCredentials(ctx context.Context, token string, isRobot bool) (*authv3.CheckResponse, error) {
	// Google OAuth access tokens start with ya29. and must be verified as OAuth tokens,
	// not JWTs (even if they contain dots matching loose regex).
	if strings.HasPrefix(token, "ya29.") {
		return s.verifyOAuthToken(ctx, token, isRobot)
	}

	if isRobot {
		resp, err := s.verifyJWTInternal(ctx, token)
		if err == nil {
			return resp, nil
		}
		resp, err = s.verifyOAuthTokenInternal(ctx, token, true)
		if err == nil {
			return resp, nil
		}
		return deniedResponse(typev3.StatusCode_BadRequest, codes.InvalidArgument, "invalid token format"), nil
	}

	// Human request
	resp, err := s.verifyOAuthTokenInternal(ctx, token, false)
	if err == nil {
		return resp, nil
	}

	return deniedResponse(typev3.StatusCode_BadRequest, codes.InvalidArgument, "invalid token format"), nil
}

func isRobot(req *authv3.CheckRequest) bool {
	if req.GetAttributes() == nil {
		return false
	}

	if ext := req.GetAttributes().GetContextExtensions(); ext != nil {
		if val, ok := ext["robot"]; ok {
			if b, ok := parseBoolValue(val); ok {
				return b
			}
		}
		if val, ok := ext["robots"]; ok {
			if b, ok := parseBoolValue(val); ok {
				return b
			}
		}
		if p, ok := ext["path"]; ok && p != "" && (strings.Contains(p, "robot") || strings.Contains(p, "robots")) {
			if b, ok := getRobotFromQuery(p); ok {
				return b
			}
		}
	}

	httpReq := req.GetAttributes().GetRequest().GetHttp()
	if httpReq != nil {
		if val := getHeader(httpReq.GetHeaders(), "x-crc-tv-robots"); val != "" {
			if b, ok := parseBoolValue(val); ok {
				return b
			}
		}
		if reqPath := httpReq.GetPath(); reqPath != "" && (strings.Contains(reqPath, "robot") || strings.Contains(reqPath, "robots")) {
			if b, ok := getRobotFromQuery(reqPath); ok {
				return b
			}
		}
	}
	return false
}

func parseBoolValue(val string) (bool, bool) {
	v := strings.ToLower(strings.TrimSpace(val))
	if v == "true" || v == "1" || v == "yes" {
		return true, true
	}
	if v == "false" || v == "0" || v == "no" {
		return false, true
	}
	return false, false
}

func getRobotFromQuery(rawURL string) (bool, bool) {
	if rawURL == "" || !strings.Contains(rawURL, "?") {
		return false, false
	}
	if u, err := url.Parse(rawURL); err == nil {
		q := u.Query()
		if val := q.Get("robots"); val != "" {
			if b, ok := parseBoolValue(val); ok {
				return b, true
			}
		}
		if val := q.Get("robot"); val != "" {
			if b, ok := parseBoolValue(val); ok {
				return b, true
			}
		}
	}
	return false, false
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
