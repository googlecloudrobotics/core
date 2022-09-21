package jwt

import (
	"encoding/base64"
	"encoding/json"
	"strings"

	jwt "github.com/form3tech-oss/jwt-go"
	"github.com/pkg/errors"
)

type payload struct {
	Aud    string
	Iss    string
	Exp    int64
	Scopes string
	Claims string
}

// PayloadUnsafe returns the unverified payload section of a given JWT.
//
// Unsafe because the content can not be trusted if you do not also verify
// the signature of the JWT.
func PayloadUnsafe(token string) (*payload, error) {
	parts := strings.Split(token, ".")
	if len(parts) != 3 {
		return nil, errors.New("invalid JWT, token must have 3 parts")
	}
	payloadBytes, err := base64.RawURLEncoding.DecodeString(parts[1])
	if err != nil {
		return nil, errors.Wrap(err, "failed to decode JWT payload section")
	}
	dat := payload{}
	err = json.Unmarshal(payloadBytes, &dat)
	if err != nil {
		return nil, errors.Wrap(err, "failed to unmarshal JWT payload section")
	}
	return &dat, nil
}

// Verify the given encoded JWT with the RSA public key in PEM format.
func VerifySignature(token string, pubKey string) error {
	key, err := jwt.ParseRSAPublicKeyFromPEM([]byte(pubKey))
	if err != nil {
		return errors.Wrap(err, "failed to parse public key")
	}
	_, err = jwt.Parse(token, func(t *jwt.Token) (interface{}, error) {
		if _, ok := t.Method.(*jwt.SigningMethodRSA); !ok {
			return nil, errors.New("unexpected signing method, only RSA family is accepted")
		}
		return key, nil
	})
	if err != nil {
		return errors.Wrap(err, "failed to parse and verify signature")
	}
	return nil
}
