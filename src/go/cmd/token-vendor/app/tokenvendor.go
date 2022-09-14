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

	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/oauth"
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
	repo PubKeyRepository
	v    *oauth.TokenVerifier
}

func NewTokenVendor(ctx context.Context, repo PubKeyRepository, v *oauth.TokenVerifier) (*TokenVendor, error) {
	return &TokenVendor{repo: repo, v: v}, nil
}

func (tv *TokenVendor) PublishPublicKey(ctx context.Context, deviceID, publicKey string) error {
	return tv.repo.PublishKey(ctx, deviceID, publicKey)
}

func (tv *TokenVendor) ReadPublicKey(ctx context.Context, deviceID string) (string, error) {
	return tv.repo.LookupKey(ctx, deviceID)
}

func (tv *TokenVendor) GetOAuth2Token(ctx context.Context, jwt string) (*GCPTokenResponse, error) {
	panic("not implemented")
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
