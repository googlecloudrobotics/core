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
	"log"
	"net/http"
	"path"

	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/api"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/app"
)

type HandlerContext struct {
	tv *app.TokenVendor
}

func NewHandlerContext(tv *app.TokenVendor) *HandlerContext {
	return &HandlerContext{tv}
}

func (h *HandlerContext) publicKeyReadHandler(w http.ResponseWriter, r *http.Request) {
	api.ErrResponse(w, http.StatusInternalServerError, "not implemented yet")
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
