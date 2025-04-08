// Copyright 2022 The Cloud Robotics Authors
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
	"bytes"
	"encoding/json"
	"encoding/pem"
	"fmt"
	"io"
	"log/slog"
	"net/http"
	"net/url"
	"path"
	"regexp"
	"strings"

	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/api"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/app"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/oauth"
	"github.com/googlecloudrobotics/ilog"
)

const (
	paramDeviceID = "device-id"
	contentType   = "content-type"
	pemFile       = "application/x-pem-file"
)

type HandlerContext struct {
	tv *app.TokenVendor
}

func NewHandlerContext(tv *app.TokenVendor) *HandlerContext {
	return &HandlerContext{tv}
}

// getQueryParam extracts a query parameter from the request URL.
//
// Multiple parameters with the same key are considered undefined and will result in error
func getQueryParam(u *url.URL, param string) (string, error) {
	values, ok := u.Query()[param]
	if !ok || len(values) != 1 {
		err := fmt.Errorf("missing or multiple query parameter %s", param)
		return "", err
	}
	return values[0], nil
}

// Handle requests to read a device's public key by device identifier.
//
// Method: GET
// URL parameter: device-id, the string identifier of the device
//
// Response code: 200 (even if key not found)
// Response body: A single public key or "" if no key was found.
func (h *HandlerContext) publicKeyReadHandler(w http.ResponseWriter, r *http.Request) {
	// validate request and parameters
	if r.Method != http.MethodGet {
		api.ErrResponse(w, http.StatusBadRequest,
			fmt.Sprintf("method %s not allowed, only %s", r.Method, http.MethodGet))
		return
	}
	deviceID, err := getQueryParam(r.URL, paramDeviceID)
	if err != nil {
		api.ErrResponse(w, http.StatusBadRequest, err.Error())
		return
	}
	if !app.IsValidDeviceID(deviceID) {
		api.ErrResponse(w, http.StatusBadRequest, "invalid device id")
		return
	}
	// retrieve public key from key repository
	publicKey, err := h.tv.ReadPublicKey(r.Context(), deviceID)
	if err != nil {
		api.ErrResponse(w, http.StatusInternalServerError, "request to repository failed")
		slog.Error("request to repository failed", ilog.Err(err))
		return
	}
	// for missing public keys (publicKey == "") we return 200 with
	// empty body for conformance with the original token vendor API.
	w.Header().Add(contentType, pemFile)
	w.Write([]byte(publicKey))
}

// Handle requests to publish the public key of a given device identifier.
//
// Method: POST
// URL parameter: device-id, the identifier of the device
// Request body: a single public key to publish
// Response code: 200 if publish succeeded
func (h *HandlerContext) publicKeyPublishHandler(w http.ResponseWriter, r *http.Request) {
	// validate request and parameters
	if r.Method != http.MethodPost {
		api.ErrResponse(w, http.StatusBadRequest,
			fmt.Sprintf("method %s not allowed, only %s", r.Method, http.MethodPost))
		return
	}
	deviceID, err := getQueryParam(r.URL, paramDeviceID)
	if err != nil {
		api.ErrResponse(w, http.StatusBadRequest, err.Error())
		return
	}
	if !app.IsValidDeviceID(deviceID) {
		api.ErrResponse(w, http.StatusBadRequest, "invalid device id")
		return
	}
	body, err := io.ReadAll(r.Body)
	if err != nil {
		api.ErrResponse(w, http.StatusInternalServerError, "failed to read request body")
		return
	}
	_, err = isValidPublicKey(body)
	if err != nil {
		api.ErrResponse(w, http.StatusBadRequest, fmt.Sprintf("public key format error %v", err))
		return
	}
	// publish the key
	err = h.tv.PublishPublicKey(r.Context(), deviceID, string(body))
	if err != nil {
		api.ErrResponse(w, http.StatusInternalServerError, "publish key failed")
		slog.Error("publish key failed", ilog.Err(err))
		return
	}
}

// isValidPublicKeyFormat validates the given public key in PEM format.
//
// The returned error provides details on why the validation failed.
func isValidPublicKey(pk []byte) (bool, error) {
	const minSize, maxSize = 100, 18000 // educated guesses, technically unlimited
	if len(pk) < minSize || len(pk) > maxSize {
		return false, fmt.Errorf("invalid key size, assert %d <= %d <= %d", minSize, len(pk), maxSize)
	}
	pk = bytes.TrimSpace(pk)
	var pkStart = []byte("-----BEGIN ")
	if !bytes.HasPrefix(pk, pkStart) {
		return false, fmt.Errorf("public key suffix %q missing", pk)
	}
	block, extraData := pem.Decode(pk)
	if len(extraData) > 0 {
		return false, fmt.Errorf("public key contains extra data (%d Bytes)", len(extraData))
	}
	if block == nil || block.Type != "PUBLIC KEY" {
		return false, fmt.Errorf("failed to decode PEM block expecting public key")
	}
	return true, nil
}

// Handle requests to retrieve a GCP access token for access to cloud resources.
//
// The robot identifies itself using its private key to sign a JWT. The JWT is
// verified by the token vendor using the public key from the key repository and
// the device identifier from the `iss` key from the JWT's payload section.
// After successful verification, the token vendor uses its own IAM identity to
// generate an access token for the `robot-service` service account and returns
// the access token to the robot. We only accept RSA as signing algorithm right
// now. This is the default used by the metadata server's oauth package [1].
//
// [1] https://github.com/golang/oauth2/blob/f21342109be17cd214ecfcd33065b79cd571673e/jwt/jwt.go#L29
//
// Method: POST
// The body is formatted like an URL query with two parameters:
//   - grant_type=urn:ietf:params:oauth:grant-type:jwt-bearer
//   - assertion=<jwt signed by robots private key>
//
// Signed JWT expected header:
// “`json
// {
//
//	"alg": "RS256",
//	"typ": "JWT"
//
// }
// “`
// The JWT body is expected to look like this:
// “`json
// {
//
//	"aud": "<accepted audience>" or "<accepted audience>?token_type=access_token", // has to match the one specified in the token vendor config
//	"iss": "<device identifier>",
//	"exp": <expiration timestamp>,
//	"scopes": "...",  // unused
//	"claims": "..." // unused
//
// }
// The response body will look like this:
// "`json
// {
//
//	"access_token":"foo", // the cloud access token
//	"expires_in":3600, // the cloud access token expiration in seconds from now
//	"scope":"http://example1.com http://example2.com", // GCP API scope URLs from token vendor config
//	"token_type":"Bearer" // static
//
// }
// “`
func (h *HandlerContext) tokenOAuth2Handler(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		api.ErrResponse(w, http.StatusBadRequest,
			fmt.Sprintf("method %s not allowed, only %s", r.Method, http.MethodPost))
		return
	}
	body, err := io.ReadAll(r.Body)
	if err != nil {
		api.ErrResponse(w, http.StatusInternalServerError, "error reading request body")
		slog.Error("error reading request body", ilog.Err(err))
		return
	}
	values, err := url.ParseQuery(string(body))
	if err != nil {
		api.ErrResponse(w, http.StatusBadRequest, err.Error())
		return
	}
	const paramGrant = "grant_type"
	const jwtGrant = "urn:ietf:params:oauth:grant-type:jwt-bearer"
	grant := values.Get(paramGrant)
	if grant != jwtGrant {
		api.ErrResponse(w, http.StatusBadRequest,
			fmt.Sprintf(`expected "%s=%s" in body`, paramGrant, jwtGrant))
		return
	}
	const paramAssert = "assertion"
	assertion := values.Get(paramAssert)
	if _, err := isValidJWT(assertion); err != nil {
		api.ErrResponse(w, http.StatusBadRequest,
			fmt.Sprintf(`expected "%s=<jwt>" in body, invalid token format: %v`, paramAssert, err))
		return
	}
	const paramServiceAccount = "service-account"
	serviceAccount := values.Get(paramServiceAccount)
	token, err := h.tv.GetOAuth2Token(r.Context(), assertion, serviceAccount)
	if err != nil {
		api.ErrResponse(w, http.StatusForbidden, "unable to retrieve cloud access token with given JWT")
		slog.Error("unable to retrieve cloud access token with given JWT", ilog.Err(err))
		return
	}
	tokenBytes, err := json.Marshal(token)
	if err != nil {
		api.ErrResponse(w, http.StatusInternalServerError, "failed to marshal upstream response")
		slog.Error("failed to marshal upstream response", ilog.Err(err))
		return
	}
	w.Header().Add(contentType, "application/json")
	w.Write(tokenBytes)
}

// Robots sign JWTs with their local private keys. These get verified against the
// public keys from the keystore. If the key is present and enabled, the token
// vendor will return status code 200.
// This endpoint allows 3rd parties to do a check against the token-vendor before
// the client reached the token vendor to retrieve an OAuth token.
// It only validates whether the robot is known to the token vendor, there is no
// further authentication or authorization done with this endpoint.
//
// URL: /apis/core.token-vendor/v1/jwt.verify
// Method: GET
// Headers:
// - Authorization: JWT that allows authorization
// Response: only http status code
func (h *HandlerContext) verifyJWTHandler(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		api.ErrResponse(w, http.StatusMethodNotAllowed,
			fmt.Sprintf("method %s not allowed, only %s", r.Method, http.MethodGet))
		return
	}

	authHeader, ok := r.Header["Authorization"]
	if !ok {
		api.ErrResponse(w, http.StatusBadRequest,
			"request did not provide Authorization header")
		return
	}

	if len(authHeader) != 1 {
		api.ErrResponse(w, http.StatusBadRequest,
			fmt.Sprintf("%q auth headers provided. Only 1 allowed", len(authHeader)))
		return
	}

	// Be slightly permissive here. Allow both forms
	// Authorization: Bearer ...
	// Authorization: ...
	jwtString := strings.TrimPrefix(authHeader[0], "Bearer ")

	if _, _, err := h.tv.ValidateJWT(r.Context(), jwtString); err != nil {
		slog.WarnContext(r.Context(), "JWT failed validation", ilog.Err(err))
		api.ErrResponse(w, http.StatusForbidden, "JWT not valid")
		return
	}
}

// Handle requests to verify if a given token has cloud access.
//
// The token is verified by testing if the token has `iam.serviceAccounts.actAs`
// authorization on either the `humanacl` or `robot-service` account by
// calling the GCP testIamPermissions API. This permission can be granted through e.g. the
// Service Account User (roles/iam.serviceAccountUser) role.
//
// Method: GET
// URL Parameters:
// - robots (optional): "true" to verify against `robot-service` role, else `humanacl`
// - token (optional): access token, if not given via header
// Headers (optional): X_FORWARDED_ACCESS_TOKEN or AUTHORIZATION
// See function `tokenFromRequest` for details on how to supply the token.
func (h *HandlerContext) verifyTokenHandler(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		api.ErrResponse(w, http.StatusBadRequest,
			fmt.Sprintf("method %s not allowed, only %s", r.Method, http.MethodGet))
		return
	}
	robots := testForRobotACL(r.URL)
	token, err := tokenFromRequest(r.URL, &r.Header)
	if err != nil {
		api.ErrResponse(w, http.StatusBadRequest, err.Error())
		return
	}
	err = h.tv.VerifyToken(r.Context(), oauth.Token(token), robots)
	if err != nil {
		api.ErrResponse(w, http.StatusForbidden, "unable to verify token")
		slog.Error("unable to verify token", ilog.Err(err))
		return
	}
	w.Write([]byte("OK"))
}

// testForRobotACL determines if the "robots" parameter is set.
func testForRobotACL(u *url.URL) bool {
	robots, err := getQueryParam(u, "robots")
	if err != nil || robots != "true" {
		return false
	}
	return true
}

// tokenFromRequest extracts the access token from the request.
//
// The access token can be supplied in one of the following ways, checked in the
// give order. This is based on the specification of the original java token vendor.
// 1. Header `X-Forwarded-Access-Token`: Token without prefix
// 2. Header `Authorization`: Token with "Bearer " prefix
// 3. URL Parameter `token`
func tokenFromRequest(u *url.URL, h *http.Header) (string, error) {
	const fwdToken = "X-Forwarded-Access-Token"
	if t := h.Get(fwdToken); t != "" {
		if _, err := isValidToken(t); err != nil {
			return "", err
		}
		return t, nil
	}
	const authHeader, authPrefix = "Authorization", "Bearer "
	if t := h.Get(authHeader); t != "" {
		if !strings.HasPrefix(t, authPrefix) {
			return "", fmt.Errorf("token in header %q has no prefix %q",
				authHeader, authPrefix)
		}
		t = strings.TrimPrefix(t, authPrefix)
		if _, err := isValidToken(t); err != nil {
			return "", err
		}
		return t, nil
	}
	const paramToken = "token"
	t, err := getQueryParam(u, paramToken)
	if err != nil {
		return "", fmt.Errorf("no token in headers %q or %q and unable to get token from URL param %q: %v",
			fwdToken, authHeader, paramToken, err)
	}
	_, err = isValidToken(t)
	if err != nil {
		return "", err
	}
	return t, nil
}

const tokenRegex = `^ya29\.[a-zA-Z0-9\.\-_]+$`

var tokenMatch = regexp.MustCompile(tokenRegex).MatchString

// isValidToken verifies the format of the token string.
//
// The returned error provides details on why the validation failed.
func isValidToken(token string) (bool, error) {
	// Minimum length is dummy, maximum from documentation
	// Source: https://cloud.google.com/iam/docs/reference/sts/rest/v1/TopLevel/token#response-body
	const minSize, maxSize = 100, 12288
	if len(token) < minSize || len(token) > maxSize {
		return false, fmt.Errorf("invalid token size, assert %d <= %d <= %d",
			minSize, len(token), maxSize)
	}
	if !tokenMatch(token) {
		return false, fmt.Errorf("token failed validation against %q", tokenRegex)
	}
	return true, nil
}

const jwtRegex = `^[a-zA-Z0-9\.\-_]+\.[a-zA-Z0-9\.\-_]+\.[a-zA-Z0-9\.\-_]+$`

var jwtMatch = regexp.MustCompile(jwtRegex).MatchString

// isValidJWT verifies the format of an encoded JWT.
//
// The returned error provides details on why the validation failed.
func isValidJWT(jwt string) (bool, error) {
	const minSize, maxSize = 100, 5000 // guess
	if len(jwt) < minSize || len(jwt) > maxSize {
		return false, fmt.Errorf("invalid size, assert %d <= %d <= %d",
			minSize, len(jwt), maxSize)
	}
	if !jwtMatch(jwt) {
		return false, fmt.Errorf("jwt failed validation against %q", jwtRegex)
	}
	return true, nil
}

// Register the API V1 API handler functions to the default http.DefaultServeMux
func Register(tv *app.TokenVendor, prefix string) error {

	slog.Debug("mounting API V1", slog.String("Prefix", prefix))

	h := NewHandlerContext(tv)

	http.HandleFunc(path.Join(prefix, "public-key.read"), h.publicKeyReadHandler)
	http.HandleFunc(path.Join(prefix, "public-key.publish"), h.publicKeyPublishHandler)
	http.HandleFunc(path.Join(prefix, "token.oauth2"), h.tokenOAuth2Handler)
	http.HandleFunc(path.Join(prefix, "token.verify"), h.verifyTokenHandler)
	http.HandleFunc(path.Join(prefix, "jwt.verify"), h.verifyJWTHandler)

	return nil
}
