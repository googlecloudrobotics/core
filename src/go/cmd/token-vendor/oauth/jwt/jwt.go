package jwt

import (
	"encoding/base64"
	"encoding/json"
	"errors"
	"fmt"
	"strings"
	"time"

	"github.com/golang-jwt/jwt/v5"
)

// clockSkewLeeway is the tolerance applied when validating the time-based
// claims (exp, nbf, iat) of a JWT. Robots may have clocks that are slightly
// out of sync with the token vendor, eg after being offline for a while, which
// would otherwise cause "token used before issued" errors.
const clockSkewLeeway = 5 * time.Minute

type payload struct {
	Aud    string
	Iss    string
	Exp    int64
	Sub    string
	Scopes string
	Claims string
}

// PayloadUnsafe returns the unverified payload section of a given JWT.
//
// Unsafe because the content can not be trusted if you do not also verify
// the signature of the JWT.
func PayloadUnsafe(jwtk string) (*payload, error) {
	parts := strings.Split(jwtk, ".")
	if len(parts) != 3 {
		return nil, errors.New("invalid JWT, token must have 3 parts")
	}
	payloadBytes, err := base64.RawURLEncoding.DecodeString(parts[1])
	if err != nil {
		return nil, fmt.Errorf("failed to decode JWT payload section: %w", err)
	}
	dat := payload{}
	err = json.Unmarshal(payloadBytes, &dat)
	if err != nil {
		return nil, fmt.Errorf("failed to unmarshal JWT payload section: %w", err)
	}
	return &dat, nil
}

// Verify the given encoded JWT with the RSA public key in PEM format.
func VerifySignature(jwtk string, pubKey string) error {
	key, err := jwt.ParseRSAPublicKeyFromPEM([]byte(pubKey))
	if err != nil {
		return fmt.Errorf("failed to parse public key: %w", err)
	}
	_, err = jwt.Parse(jwtk, func(t *jwt.Token) (any, error) {
		if _, ok := t.Method.(*jwt.SigningMethodRSA); !ok {
			return nil, errors.New("unexpected signing method, only RSA family is accepted")
		}
		return key, nil
	},
		// This list matches *jwt.SigningMethodRSA. Robots sign with RS256.
		jwt.WithValidMethods([]string{"RS256", "RS384", "RS512"}),
		jwt.WithIssuedAt(),
		jwt.WithLeeway(clockSkewLeeway),
	)
	if err != nil {
		return fmt.Errorf("failed to parse and verify signature: %w", err)
	}
	return nil
}
