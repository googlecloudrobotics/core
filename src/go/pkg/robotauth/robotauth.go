// Copyright 2019 The Cloud Robotics Authors
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

// The robotauth package contains the class for reading and writing the
// robot-id.json file. This file contains the id & private key of a robot
// that's connected to a Cloud project.
package robotauth

import (
	"bytes"
	"context"
	"crypto/rand"
	"crypto/rsa"
	"crypto/x509"
	"encoding/json"
	"encoding/pem"
	"fmt"
	"io"
	"net/http"
	"os"
	"path/filepath"
	"time"

	"github.com/googlecloudrobotics/core/src/go/pkg/kubeutils"
	"golang.org/x/oauth2"
	"golang.org/x/oauth2/jws"
	"golang.org/x/oauth2/jwt"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/client-go/kubernetes"
)

const (
	// TODO(ensonic): setup-dev creates a key and stores it, only for the ssh-app to read it
	credentialsFile = "~/.config/cloud-robotics/robot-id.json"
)

// Object containing ID, as stored in robot-id.json.
type RobotAuth struct {
	RobotName           string `json:"id"`
	ProjectId           string `json:"project_id"`
	PublicKeyRegistryId string `json:"public_key_registry_id"`
	PrivateKey          []byte `json:"private_key"`
	Domain              string `json:"domain"`
}

func filename() string {
	return kubeutils.ExpandUser(credentialsFile)
}

// LoadFromFile loads key from json file. If keyfile is "", it tries to load
// from the default location.
func LoadFromFile(keyfile string) (*RobotAuth, error) {
	if keyfile == "" {
		keyfile = filename()
	}
	raw, err := os.ReadFile(keyfile)
	if err != nil {
		return nil, fmt.Errorf("failed to read %v: %v", credentialsFile, err)
	}

	var robotAuth RobotAuth
	err = json.Unmarshal(raw, &robotAuth)
	if err != nil {
		return nil, fmt.Errorf("failed to parse %v: %v", credentialsFile, err)
	}

	return &robotAuth, nil
}

func LoadFromK8sSecret(ctx context.Context, clientset kubernetes.Interface, namespace string) (*RobotAuth, error) {
	s, err := clientset.CoreV1().Secrets(namespace).Get(ctx, "robot-auth", metav1.GetOptions{})
	if err != nil {
		return nil, err
	}
	encoded, ok := s.Data["json"]
	if !ok {
		return nil, fmt.Errorf("could not find json key in secret's data")
	}
	var ret RobotAuth
	if err := json.NewDecoder(bytes.NewReader(encoded)).Decode(&ret); err != nil {
		return nil, err
	}

	return &ret, nil
}

// StoreInFile writes a newly-chosen ID to disk.
func (r *RobotAuth) StoreInFile() error {
	raw, err := json.Marshal(r)
	if err != nil {
		return fmt.Errorf("failed to serialize ID: %v", err)
	}

	file := filename()
	if err := os.MkdirAll(kubeutils.ExpandUser(filepath.Dir(file)), 0700); err != nil {
		return err
	}

	err = os.WriteFile(file, raw, 0600)
	if err != nil {
		return fmt.Errorf("failed to write %v: %v", credentialsFile, err)
	}

	return nil
}

// StoreInK8sSecret writes new robot-id to kubernetes secret.
func (r *RobotAuth) StoreInK8sSecret(ctx context.Context, clientset kubernetes.Interface, namespace string) error {
	raw, err := json.Marshal(r)
	if err != nil {
		return fmt.Errorf("failed to serialize ID: %v", err)
	}

	return kubeutils.UpdateSecret(ctx, clientset, &corev1.Secret{
		ObjectMeta: metav1.ObjectMeta{
			Name:      "robot-auth",
			Namespace: namespace,
		},
		Type: corev1.SecretTypeOpaque,
		Data: map[string][]byte{
			"json": raw,
		},
	})
}

// CreatePrivateKey creates a private key.
// The private key is written to the RobotAuth struct.
func (r *RobotAuth) CreatePrivateKey() error {
	key, err := rsa.GenerateKey(rand.Reader, 2048)
	if err != nil {
		return err
	}
	pkcs8, err := x509.MarshalPKCS8PrivateKey(key)
	if err != nil {
		return err
	}
	r.PrivateKey = pem.EncodeToMemory(&pem.Block{
		Type:  "PRIVATE KEY",
		Bytes: pkcs8,
	})
	return nil
}

func (r *RobotAuth) getTokenEndpoint() string {
	return fmt.Sprintf("https://%s/apis/core.token-vendor/v1/token.oauth2", r.Domain)
}

// CreateRobotTokenSource creates an OAuth2 token source for the token vendor.
// This token source returns Google Cloud access token minted for the robot-service@
// service account.
func (r *RobotAuth) CreateRobotTokenSource(ctx context.Context) oauth2.TokenSource {
	c := jwt.Config{
		// Will be used as "issuer" of the outgoing JWT. Is not formatted as an email though
		Email:      r.PublicKeyRegistryId,
		Expires:    time.Minute * 30,
		PrivateKey: r.PrivateKey,
		Scopes:     []string{},
		// TODO: shouldn't this be the service-account name we want to get the token for?
		Subject:  r.RobotName,
		TokenURL: r.getTokenEndpoint(),
	}
	return c.TokenSource(ctx)
}

// CreateJWT allows to create a JWT for authentication against the token vendor.
// This does not grant Google Cloud access, but can be used for for explicit
// authentication with the token vendor.
func (r *RobotAuth) CreateJWT(ctx context.Context, lifetime time.Duration) (string, error) {
	p, _ := pem.Decode(r.PrivateKey)
	if p == nil {
		return "", fmt.Errorf("decode private key")
	}
	parsedKey, err := x509.ParsePKCS8PrivateKey(p.Bytes)
	if err != nil {
		parsedKey, err = x509.ParsePKCS1PrivateKey(p.Bytes)
		if err != nil {
			return "", fmt.Errorf("private key should be a PEM or plain PKCS1 or PKCS8; parse error: %v", err)
		}
	}
	parsed, ok := parsedKey.(*rsa.PrivateKey)
	if !ok {
		return "", fmt.Errorf("private key is invalid")
	}

	// We re-use the token audience here.
	// While it would be nicer to use a specific token.verify endpoint here,
	// the token-vendor takes a full path it verifies.
	// This would allow this token to be used for getting an OAuth token, but
	// the token and identity endpoints use the same access protection,
	// so there's no functional difference either way.
	claimSet := &jws.ClaimSet{
		Iss: r.PublicKeyRegistryId,
		Aud: r.getTokenEndpoint(),
		Sub: r.RobotName,
		Prn: r.RobotName,
		Exp: time.Now().Add(lifetime).Unix(),
	}

	ret, err := jws.Encode(&jws.Header{Algorithm: "RS256", Typ: "JWT"}, claimSet, parsed)
	if err != nil {
		return "", err
	}

	return ret, nil
}

// robotJWTSource gets robot JWTs from the metadata-server.
type robotJWTSource struct {
	client http.Client
}

// Token gets a robot JWT from the metadata-server or returns an error.
// Token must be safe for concurrent use by multiple goroutines.
// The returned Token must not be modified.
func (ts *robotJWTSource) Token() (*oauth2.Token, error) {
	req, err := http.NewRequest(http.MethodGet, "http://169.254.169.254/computeMetadata/v1/instance/service-accounts/default/identity", nil)
	if err != nil {
		return nil, err
	}
	resp, err := ts.client.Do(req)
	if err != nil {
		return nil, err
	}
	defer resp.Body.Close()

	// Read body before checking status to ensure connection can be reused.
	tokenBytes, err := io.ReadAll(resp.Body)
	if err != nil {
		return nil, fmt.Errorf("read response body: %w", err)
	}
	tokenString := string(tokenBytes)
	if resp.StatusCode != http.StatusOK {
		return nil, fmt.Errorf("status code %d, body:\n%s", resp.StatusCode, tokenString)
	}

	// Decode token so we can set the expiry and the client knows when to
	// refresh. No need to check the signature here (onprem) as it will be
	// checked by the cloud backend.
	claims, err := jws.Decode(tokenString)
	if err != nil {
		return nil, fmt.Errorf("decode: %w", err)
	}
	return &oauth2.Token{
		TokenType:   "Bearer",
		AccessToken: tokenString,
		Expiry:      time.Unix(claims.Exp, 0),
	}, nil
}

const (
	// Minimum remaining lifetime of JWTs from CreateJWTSource().
	jwtMinLifetime = time.Minute
)

// CreateJWTSource creates an OAuth2 token source for the JWTs signed by the
// robot's private key.
func CreateJWTSource() oauth2.TokenSource {
	ts := &robotJWTSource{}
	return oauth2.ReuseTokenSourceWithExpiry(nil, ts, jwtMinLifetime)
}
