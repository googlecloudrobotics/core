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
	"fmt"
	"log"
	"net/http"
	"path"
	"regexp"

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

// getQueryParam extracts a query parameter from the request.
//
// Multiple parameters with the same key are considered undefined and will result in error
func getQueryParam(r *http.Request, param string) (string, error) {
	values, ok := r.URL.Query()[param]
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
	deviceID, err := getQueryParam(r, paramDeviceID)
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

func (h *HandlerContext) publicKeyPublishHandler(w http.ResponseWriter, r *http.Request) {
	api.ErrResponse(w, http.StatusInternalServerError, "not implemented yet")
}

func (h *HandlerContext) tokenOAuth2Handler(w http.ResponseWriter, r *http.Request) {
	api.ErrResponse(w, http.StatusInternalServerError, "not implemented yet")
}

func (h *HandlerContext) verifyTokenHandler(w http.ResponseWriter, r *http.Request) {
	api.ErrResponse(w, http.StatusInternalServerError, "not implemented yet")
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
