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

package app

import (
	"context"
	"fmt"
	"regexp"
	"strings"
	"time"

	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/oauth"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/oauth/jwt"
	"github.com/pkg/errors"
)

type GCPTokenResponse struct {
	AccessToken  string `json:"access_token"`
	ExpiresInSec int    `json:"expires_in"`
	TokenType    string `json:"token_type"`
	Scope        string `json:"scope"`
}

type PubKeyRepository interface {
	LookupKey(ctx context.Context, deviceID string) (string, error)
	PublishKey(ctx context.Context, deviceID, publicKey string) error
}

type TokenVendor struct {
	repo   PubKeyRepository
	v      *oauth.TokenVerifier
	accAud string
}

func NewTokenVendor(ctx context.Context, repo PubKeyRepository, v *oauth.TokenVerifier, acceptedAudience string) (*TokenVendor, error) {
	return &TokenVendor{repo: repo, v: v, accAud: acceptedAudience}, nil
}

func (tv *TokenVendor) PublishPublicKey(ctx context.Context, deviceID, publicKey string) error {
	return tv.repo.PublishKey(ctx, deviceID, publicKey)
}

func (tv *TokenVendor) ReadPublicKey(ctx context.Context, deviceID string) (string, error) {
	return tv.repo.LookupKey(ctx, deviceID)
}

func (tv *TokenVendor) GetOAuth2Token(ctx context.Context, token string) (*GCPTokenResponse, error) {
	p, err := jwt.PayloadUnsafe(token)
	if err != nil {
		return nil, errors.Wrap(err, "failed to extract JWT payload")
	}
	exp := time.Unix(p.Exp, 0)
	if exp.Before(time.Now()) {
		return nil, errors.New("JWT has expired")
	}
	if err := acceptedAudience(p.Aud, tv.accAud); err != nil {
		return nil, errors.Wrap(err, "validation of JWT audience failed")
	}
	if !IsValidDeviceID(p.Iss) {
		return nil, errors.New("missing or invalid device identifier (`iss` key)")
	}
	deviceId := p.Iss
	pubKey, err := tv.repo.LookupKey(ctx, deviceId)
	if err != nil {
		return nil, errors.Wrapf(err, "failed to retrieve public key for device %q", deviceId)
	}
	err = jwt.VerifySignature(token, pubKey)
	if err != nil {
		return nil, errors.Wrap(err, "failed to verify signature")
	}
	//TODO(csieber): Missing implementation for generating the access token
	return nil, errors.New("not implemented yet")
}

// acceptedAudience validates JWT audience as defined by Java-based token vendor
//
// `aud` is the value of the audience key from the JWT and `accAud` the configured
// accepted audience of the token vendor.
func acceptedAudience(aud, accAud string) error {
	qualified := accAud + "?token_type=access_token"
	auds := strings.Split(aud, " ")
	if !contains(auds, accAud) && !contains(auds, qualified) {
		return fmt.Errorf("audience must contain %q or %q", accAud, qualified)
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

func (tv *TokenVendor) VerifyToken(ctx context.Context, token oauth.Token, robots bool) error {
	var acl string
	if robots {
		acl = "robot-service"
	} else {
		acl = "human-acl"
	}
	return tv.v.Verify(ctx, token, acl)
}

// RFC952 hostnames
var isValidDeviceIDRegex = regexp.MustCompile(`^[a-zA-Z]([a-zA-Z0-9\-]+[\.]?)*[a-zA-Z0-9]$`).MatchString

// IsValidDeviceID validates the given identifier for string length and characters used
//
// Validation is based on the RFC952 for hostnames.
func IsValidDeviceID(ID string) bool {
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
