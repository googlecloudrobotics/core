// Copyright 2019 The Google Cloud Robotics Authors
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

package gcr

import (
	"context"
	"encoding/json"
	"fmt"
	"log"

	"cloud-robotics.googlesource.com/cloud-robotics/pkg/kubeutils"
	"cloud-robotics.googlesource.com/cloud-robotics/pkg/robotauth"
	corev1 "k8s.io/api/core/v1"
	k8serrors "k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/client-go/kubernetes"
)

// dockercfgJSON takes a service account key, and converts it into the JSON
// format required for k8s's docker-registry secrets.
func dockercfgJSON(token string) []byte {
	type dockercfg struct {
		Username string `json:"username"`
		Password string `json:"password"`
		Email    string `json:"email"`
		Auth     []byte `json:"auth"`
	}

	m := map[string]interface{}{
		"https://eu.gcr.io": dockercfg{
			Username: "oauth2accesstoken",
			Password: string(token),
			Email:    "not@val.id",
			Auth:     []byte("oauth2accesstoken:" + token),
		},
	}
	b, err := json.Marshal(m)
	if err != nil {
		log.Fatal("unexpected error marshalling dockercfg: ", err)
	}
	return b
}

func UpdateGcrCredentials(k8s *kubernetes.Clientset, auth *robotauth.RobotAuth) error {
	ctx := context.Background()
	tokenSource := auth.CreateRobotTokenSource(ctx)
	token, err := tokenSource.Token()
	if err != nil {
		return fmt.Errorf("failed to get token: %v", err)
	}
	err = kubeutils.UpdateSecret(
		k8s,
		"gcr-json-key",
		corev1.SecretTypeDockercfg,
		map[string][]byte{
			".dockercfg": dockercfgJSON(token.AccessToken),
		})
	if err != nil {
		return fmt.Errorf("failed to update kubernetes secret: %v", err)
	}
	// Tell k8s to use this key by pointing the default SA at it.
	sa := k8s.CoreV1().ServiceAccounts(corev1.NamespaceDefault)
	patchData := []byte(`{"imagePullSecrets": [{"name": "gcr-json-key"}]}`)
	for {
		_, err = sa.Patch("default", types.StrategicMergePatchType, patchData)
		if err == nil {
			break
		} else if !k8serrors.IsNotFound(err) {
			return fmt.Errorf("failed to update kubernetes service account: %v", err)
		}
	}
	return nil
}
