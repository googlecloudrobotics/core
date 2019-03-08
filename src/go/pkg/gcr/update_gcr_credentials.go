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

/*
Library for updating the token used to pull images from GCR in the surrounding cluster.
*/
package gcr

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"time"

	"github.com/googlecloudrobotics/core/src/go/pkg/kubeutils"
	"github.com/googlecloudrobotics/core/src/go/pkg/robotauth"
	corev1 "k8s.io/api/core/v1"
	k8serrors "k8s.io/apimachinery/pkg/api/errors"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
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

	m := map[string]interface{}{}
	for _, r := range []string{"gcr.io", "asia.gcr.io", "eu.gcr.io", "us.gcr.io"} {
		m["https://"+r] = dockercfg{
			Username: "oauth2accesstoken",
			Password: string(token),
			Email:    "not@val.id",
			Auth:     []byte("oauth2accesstoken:" + token),
		}
	}
	b, err := json.Marshal(m)
	if err != nil {
		log.Fatal("unexpected error marshalling dockercfg: ", err)
	}
	return b
}

func patchServiceAccount(k8s *kubernetes.Clientset, name string, namespace string, patchData []byte) error {
	sa := k8s.CoreV1().ServiceAccounts(namespace)
	for {
		_, err := sa.Patch(name, types.StrategicMergePatchType, patchData)
		if err == nil {
			break
		} else if !k8serrors.IsNotFound(err) {
			return fmt.Errorf("failed to update kubernetes service account: %v", err)
		}
		time.Sleep(time.Second)
	}
	return nil
}

// UpdateGcrCredentials authenticates to the cloud cluster using the auth config given and updates
// the credentials used to pull images from GCR.
func UpdateGcrCredentials(k8s *kubernetes.Clientset, auth *robotauth.RobotAuth) error {
	ctx := context.Background()
	tokenSource := auth.CreateRobotTokenSource(ctx)
	token, err := tokenSource.Token()
	if err != nil {
		return fmt.Errorf("failed to get token: %v", err)
	}

	nsList, err := k8s.CoreV1().Namespaces().List(metav1.ListOptions{})
	if err != nil {
		return fmt.Errorf("failed to list namespaces: %v", err)
	}
	cfgData := map[string][]byte{".dockercfg": dockercfgJSON(token.AccessToken)}
	patchData := []byte(`{"imagePullSecrets": [{"name": "gcr-json-key"}]}`)
	haveError := false
	for _, ns := range nsList.Items {
		namespace := ns.ObjectMeta.Name
		// TODO(ensonic): do this for all namespaces (that have a 'gcr-json-key'), always do it for 'default'

		// Create a docker config with the access-token and store as secret
		err = kubeutils.UpdateSecret(k8s, "gcr-json-key", namespace, corev1.SecretTypeDockercfg,
			cfgData)
		if err != nil {
			log.Printf("failed to update kubernetes secret for namespace %s: %v", namespace, err)
			haveError = true
			continue
		}

		// Tell k8s to use this key by pointing the default SA at it.
		err = patchServiceAccount(k8s, "default", namespace, patchData)
		if err != nil {
			log.Printf("failed to update kubernetes service account for namespace %s: %v", namespace, err)
			haveError = true
		}
	}
	if haveError {
		return fmt.Errorf("failed to update one or more namespaces")
	}
	return nil
}
