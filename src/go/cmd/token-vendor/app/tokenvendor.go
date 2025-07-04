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
	"log/slog"
	"net/mail"
	"regexp"
	"strings"
	"time"

	"github.com/pkg/errors"
	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promauto"

	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/oauth"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/oauth/jwt"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/repository"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/tokensource"
	"github.com/googlecloudrobotics/ilog"
)

type TokenVendor struct {
	repo          repository.PubKeyRepository
	v             *oauth.TokenVerifier
	ts            *tokensource.GCPTokenSource
	accAud        string
	defaultSAName string
}

func NewTokenVendor(ctx context.Context, repo repository.PubKeyRepository, v *oauth.TokenVerifier, ts *tokensource.GCPTokenSource, acceptedAudience, defaultSAName string) (*TokenVendor, error) {
	if acceptedAudience == "" {
		return nil, errors.New("accepted audience must not be empty")
	}
	return &TokenVendor{repo: repo, v: v, ts: ts, accAud: acceptedAudience, defaultSAName: defaultSAName}, nil
}

func (tv *TokenVendor) PublishPublicKey(ctx context.Context, deviceID, publicKey string) error {
	slog.Info("Publishing public Key", slog.String("DeviceID", deviceID))
	return tv.repo.PublishKey(ctx, deviceID, publicKey)
}

func (tv *TokenVendor) ReadPublicKey(ctx context.Context, deviceID string) (string, error) {
	slog.Debug("Returning public Key", slog.String("DeviceID", deviceID))
	key, err := tv.repo.LookupKey(ctx, deviceID)
	if key != nil {
		return key.PublicKey, nil
	}
	return "", err
}

func (tv *TokenVendor) ConfigurePublicKey(ctx context.Context, deviceID string, opts repository.KeyOptions) error {
	if err := validateKeyOptions(opts); err != nil {
		slog.Error("Configuring public Key", ilog.Err(err))
		return err
	}
	slog.Debug("Configuring public Key", slog.String("DeviceID", deviceID))
	return tv.repo.ConfigureKey(ctx, deviceID, opts)
}

func validateKeyOptions(opts repository.KeyOptions) error {
	// Sanity check for the config values
	if opts.ServiceAccount != "" {
		if err := validateEmail(opts.ServiceAccount); err != nil {
			return fmt.Errorf("ServiceAccount field is not a valid email address: %v", err)
		}
	}
	if opts.ServiceAccountDelegate != "" {
		if err := validateEmail(opts.ServiceAccountDelegate); err != nil {
			return fmt.Errorf("ServiceAccountDelegate field is not a valid email address: %v", err)
		}
	}
	return nil
}

func validateEmail(e string) error {
	_, err := mail.ParseAddress(e)
	return err
}

var (
	tokensRequested = promauto.NewCounterVec(
		prometheus.CounterOpts{
			Name: "tokens_requested",
			Help: "Number of tokens requested",
		},
		[]string{"result"},
	)
	tokensRequestedDurations = promauto.NewHistogramVec(
		prometheus.HistogramOpts{
			Name: "tokens_requested_durations",
			Help: "Time it took to retrieve a token or fail in ms",
		},
		[]string{"result"},
	)
)

func (tv *TokenVendor) GetOAuth2Token(ctx context.Context, jwtk string) (*tokensource.TokenResponse, error) {
	ts := time.Now()
	r, err := tv.getOAuth2Token(ctx, jwtk)
	var state string
	if err != nil {
		state = "failed"
	} else {
		state = "success"
	}
	tokensRequested.WithLabelValues(state).Inc()
	tokensRequestedDurations.WithLabelValues(state).Observe(float64(time.Since(ts).Milliseconds()))
	return r, err
}

// DeviceAuth contains authorization information for device with DeviceID.
// Information is extracted from request's OAuth2 JWT Payload
type DeviceAuth struct {
	DeviceID   string
	Key        *repository.Key
	ServiceAcc string
}

// used for testing
var jwtVerifySignature = jwt.VerifySignature

func (tv *TokenVendor) ValidateJWT(ctx context.Context, jwtk string) (*DeviceAuth, error) {
	p, err := jwt.PayloadUnsafe(jwtk)
	if err != nil {
		return nil, errors.Wrap(err, "failed to extract JWT payload")
	}
	exp := time.Unix(p.Exp, 0)
	if exp.Before(time.Now()) {
		return nil, fmt.Errorf("JWT has expired %v, %v ago (iss: %q)",
			exp, time.Since(exp), p.Iss)
	}
	if err := acceptedAudience(p.Aud, tv.accAud); err != nil {
		return nil, errors.Wrapf(err, "validation of JWT audience failed (iss: %q)", p.Iss)
	}
	if !IsValidDeviceID(p.Iss) {
		return nil, fmt.Errorf("missing or invalid device identifier (`iss`: %q)", p.Iss)
	}
	deviceID := p.Iss
	k, err := tv.repo.LookupKey(ctx, deviceID)
	if err != nil {
		return nil, errors.Wrapf(err, "failed to retrieve public Key for device %q", deviceID)
	}
	if k.PublicKey == "" {
		return nil, errors.Errorf("no public Key found for device %q", deviceID)
	}
	err = jwtVerifySignature(jwtk, k.PublicKey)
	if err != nil {
		return nil, errors.Wrapf(err, "failed to verify signature for device %q", deviceID)
	}

	effectiveSub := p.Sub
	if validateEmail(p.Sub) != nil || !strings.HasSuffix(p.Sub, ".iam.gserviceaccount.com") {
		// To support legacy systems we are going to ignore subjects,
		// which cannot represent service accounts.
		effectiveSub = ""
	}

	return &DeviceAuth{
		DeviceID:   deviceID,
		Key:        k,
		ServiceAcc: effectiveSub,
	}, nil
}

func (tv *TokenVendor) getOAuth2Token(ctx context.Context, jwtk string) (*tokensource.TokenResponse, error) {
	authInfo, err := tv.ValidateJWT(ctx, jwtk)
	if err != nil {
		return nil, err
	}
	saName, err := serviceAccountName(tv.defaultSAName, authInfo.Key.SAName, authInfo.ServiceAcc)
	if err != nil {
		return nil, err
	}
	cloudToken, err := tv.ts.Token(ctx, saName, authInfo.Key.SADelegateName)
	if err != nil {
		return nil, errors.Wrapf(err, "failed to retrieve a cloud token for device %q", authInfo.DeviceID)
	}
	slog.Info("Handing out cloud token", slog.String("DeviceID", authInfo.DeviceID), slog.String("ServiceAccount", authInfo.Key.SAName))
	return cloudToken, nil
}

func serviceAccountName(saDef, saCfg, saReq string) (string, error) {
	if saReq == "" {
		// nothing given, choose default
		if saCfg != "" {
			return saCfg, nil
		}
		return saDef, nil
	}
	if saReq == saDef || saReq == saCfg {
		return saReq, nil
	}
	return "", fmt.Errorf("service account %q not allowed", saReq)
}

// acceptedAudience validates JWT audience as defined by the token vendor
//
// `aud` is the value of the audience Key from the JWT and `accAud` the configured
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

var (
	tokensVerified = promauto.NewCounterVec(
		prometheus.CounterOpts{
			Name: "tokens_verified",
			Help: "Number of tokens verified",
		},
		[]string{"acl", "result"},
	)
	tokensVerifiedDurations = promauto.NewHistogramVec(
		prometheus.HistogramOpts{
			Name: "tokens_verified_durations",
			Help: "Time it took to check a token in ms",
		},
		[]string{"acl", "result"},
	)
)

func init() {
	// pre-register label values we know
	for _, acl := range []string{"robot-service", "human-acl"} {
		for _, result := range []string{"success", "failed"} {
			tokensVerified.WithLabelValues(acl, result)
			tokensVerifiedDurations.WithLabelValues(acl, result)
		}
	}
	for _, result := range []string{"success", "failed"} {
		tokensRequested.WithLabelValues(result)
		tokensRequestedDurations.WithLabelValues(result)
	}
}

func (tv *TokenVendor) VerifyToken(ctx context.Context, token oauth.Token, robots bool) error {
	var acl string
	if robots {
		acl = "robot-service"
	} else {
		acl = "human-acl"
	}
	slog.Debug("Verifying token", slog.String("ACL", acl))
	ts := time.Now()
	err := tv.v.Verify(ctx, token, acl)
	var state string
	if err != nil {
		state = "failed"
	} else {
		state = "success"
	}
	tokensVerified.WithLabelValues(acl, state).Inc()
	tokensVerifiedDurations.WithLabelValues(acl, state).Observe(float64(time.Since(ts).Milliseconds()))
	return err
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
