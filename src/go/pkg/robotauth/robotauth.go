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
	"context"
	"crypto/rand"
	"crypto/rsa"
	"crypto/x509"
	"encoding/json"
	"encoding/pem"
	"fmt"
	"os"
	"path/filepath"
	"time"

	"github.com/googlecloudrobotics/core/src/go/pkg/kubeutils"
	"golang.org/x/oauth2"
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
func (r *RobotAuth) StoreInK8sSecret(ctx context.Context, clientset *kubernetes.Clientset, namespace string) error {
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
		Email:      r.PublicKeyRegistryId, // Will be used as "issuer" of the outgoing JWT.
		Expires:    time.Minute * 30,
		PrivateKey: r.PrivateKey,
		Scopes:     []string{r.getTokenEndpoint()},
		Subject:    r.RobotName,
		TokenURL:   r.getTokenEndpoint(),
	}
	return c.TokenSource(ctx)
}
