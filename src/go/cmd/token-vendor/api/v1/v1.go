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
	"encoding/pem"
	"fmt"
	"io"
	"log"
	"net/http"
	"net/url"
	"path"
	"regexp"
	"strings"

	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/api"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/app"
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

// RFC952 hostnames
var isValidDeviceIDRegex = regexp.MustCompile(`^[a-zA-Z]([a-zA-Z0-9\-]+[\.]?)*[a-zA-Z0-9]$`).MatchString

// isValidDeviceID validates the given identifier for string length and characters used
//
// Validation is based on the RFC952 for hostnames.
func isValidDeviceID(ID string) bool {
	const minLen, maxLen = 3, 255
	l := len(ID)
	if l < minLen || l > maxLen {
		return false
	}
	if !isValidDeviceIDRegex(ID) {
		return false
	}
	return true
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
	if !isValidDeviceID(deviceID) {
		api.ErrResponse(w, http.StatusBadRequest, "invalid device id")
		return
	}
	// retrieve public key from key repository
	publicKey, err := h.tv.ReadPublicKey(r.Context(), deviceID)
	if err != nil {
		api.ErrResponse(w, http.StatusInternalServerError, "request to repository failed")
		log.Printf("%v\n", err)
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
	if !isValidDeviceID(deviceID) {
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
		log.Printf("%v\n", err)
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

func (h *HandlerContext) tokenOAuth2Handler(w http.ResponseWriter, r *http.Request) {
	api.ErrResponse(w, http.StatusInternalServerError, "not implemented yet")
}

// Handle requests to verify if a given token has cloud access.
//
// The token is verified by testing if the token has `iam.serviceAccounts.actAs`
// authorization on either the `humanacl` or `robot-service` account by
// calling the GCP testIamPermissions API.
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
	err = h.tv.VerifyToken(r.Context(), token, robots)
	if err != nil {
		api.ErrResponse(w, http.StatusForbidden, "unable to verify token")
		log.Printf("%v\n", err)
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
// 2. Header `Authorization`: Token with "Bearer: " prefix
// 3. URL Parameter `token`
func tokenFromRequest(u *url.URL, h *http.Header) (string, error) {
	const fwdToken = "X-Forwarded-Access-Token"
	if t := h.Get(fwdToken); t != "" {
		if _, err := isValidToken(t); err != nil {
			return "", err
		}
		return t, nil
	}
	const authHeader, authPrefix = "Authorization", "Bearer: "
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

// Register the API V1 API handler functions to the default http.DefaultServeMux
func Register(tv *app.TokenVendor, prefix string) error {

	log.Printf("mounting API V1 at %s", prefix)

	h := NewHandlerContext(tv)

	http.HandleFunc(path.Join(prefix, "public-key.read"), h.publicKeyReadHandler)
	http.HandleFunc(path.Join(prefix, "public-key.publish"), h.publicKeyPublishHandler)
	http.HandleFunc(path.Join(prefix, "token.oauth2"), h.tokenOAuth2Handler)
	http.HandleFunc(path.Join(prefix, "token.verify"), h.verifyTokenHandler)

	return nil
}
