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

	log "github.com/sirupsen/logrus"

	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/oauth"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/oauth/jwt"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/tokensource"
	"github.com/pkg/errors"
)

type PubKeyRepository interface {

	// LookupKey retrieves the public key of a device from the repository.
	// An empty string return indicates that no key exists for the given identifier or
	// that the device is blocked.
	LookupKey(ctx context.Context, deviceID string) (string, error)
	PublishKey(ctx context.Context, deviceID, publicKey string) error
}

type TokenVendor struct {
	repo   PubKeyRepository
	v      *oauth.TokenVerifier
	ts     *tokensource.GCPTokenSource
	accAud string
}

func NewTokenVendor(ctx context.Context, repo PubKeyRepository, v *oauth.TokenVerifier, ts *tokensource.GCPTokenSource, acceptedAudience string) (*TokenVendor, error) {
	if acceptedAudience == "" {
		return nil, errors.New("accepted audience must not be empty")
	}
	return &TokenVendor{repo: repo, v: v, accAud: acceptedAudience, ts: ts}, nil
}

func (tv *TokenVendor) PublishPublicKey(ctx context.Context, deviceID, publicKey string) error {
	log.Info("Publishing public key for device ", deviceID)
	return tv.repo.PublishKey(ctx, deviceID, publicKey)
}

func (tv *TokenVendor) ReadPublicKey(ctx context.Context, deviceID string) (string, error) {
	log.Debug("Returning public key for device ", deviceID)
	return tv.repo.LookupKey(ctx, deviceID)
}

func (tv *TokenVendor) GetOAuth2Token(ctx context.Context, jwtk string) (*tokensource.TokenResponse, error) {
	p, err := jwt.PayloadUnsafe(jwtk)
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
	deviceID := p.Iss
	pubKey, err := tv.repo.LookupKey(ctx, deviceID)
	if err != nil {
		return nil, errors.Wrapf(err, "failed to retrieve public key for device %q", deviceID)
	}
	err = jwt.VerifySignature(jwtk, pubKey)
	if err != nil {
		return nil, errors.Wrap(err, "failed to verify signature")
	}
	cloudToken, err := tv.ts.Token(ctx)
	if err != nil {
		return nil, errors.Wrap(err, "failed to retrieve a cloud token")
	}
	log.Info("Handing out cloud token for device ", deviceID)
	return cloudToken, nil
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
	log.Debug("Verifying a token against ACL ", acl)
	return tv.v.Verify(ctx, token, acl)
}

// Regex for RFC 1123 subdomain format
// The device identifier is used as name for the Kubernetes configmap and thus is validated
// using the same regex as it is used by the Kubernetes API.
// https://kubernetes.io/docs/concepts/overview/working-with-objects/names/#dns-label-names
// https://github.com/kubernetes/kubernetes/blob/976a940f4a4e84fe814583848f97b9aafcdb083f/staging/src/k8s.io/apimachinery/pkg/util/validation/validation.go#L209
var isValidDeviceIDRegex = regexp.MustCompile(`^[a-z0-9]([-a-z0-9]*[a-z0-9])?(\.[a-z0-9]([-a-z0-9]*[a-z0-9])?)*$`).MatchString

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
