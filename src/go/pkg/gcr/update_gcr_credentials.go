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
	"log/slog"
	"os"
	"time"

	"github.com/cenkalti/backoff"
	"github.com/googlecloudrobotics/core/src/go/pkg/kubeutils"
	"github.com/googlecloudrobotics/core/src/go/pkg/robotauth"
	"github.com/googlecloudrobotics/ilog"
	corev1 "k8s.io/api/core/v1"
	k8serrors "k8s.io/apimachinery/pkg/api/errors"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/client-go/kubernetes"
)

// Name of the secret that stores the GCR pull token.
const SecretName = "gcr-json-key"

// DockerCfgJSON takes a service account key, and converts it into the JSON
// format required for k8s's docker-registry secrets.
func DockerCfgJSON(token string) []byte {
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
		slog.Error("unexpected error marshalling dockercfg", ilog.Err(err))
		os.Exit(1)
	}
	return b
}

func patchServiceAccount(ctx context.Context, k8s *kubernetes.Clientset, name string, namespace string, patchData []byte) error {
	sa := k8s.CoreV1().ServiceAccounts(namespace)
	return backoff.Retry(
		func() error {
			_, err := sa.Patch(ctx, name, types.StrategicMergePatchType, patchData, metav1.PatchOptions{})
			if err != nil && !k8serrors.IsNotFound(err) {
				return backoff.Permanent(fmt.Errorf("failed to apply %q: %v", patchData, err))
			}
			return err
		},
		// Wait up to a minute for kube-controller-manager to create
		// the SA in new namespaces. I suspect there's a race condition
		// if the namespace is created and then quickly deleted
		// (b/281647304) which might cause this to time out - if we see
		// this error showing up a lot, we could check for namespace
		// deletion instead.
		backoff.WithMaxRetries(backoff.NewConstantBackOff(time.Second), 60),
	)
}

// UpdateGcrCredentials authenticates to the cloud cluster using the auth config given and updates
// the credentials used to pull images from GCR.
func UpdateGcrCredentials(ctx context.Context, k8s *kubernetes.Clientset, auth *robotauth.RobotAuth, gcpSaChain ...string) error {
	tokenSource := auth.CreateRobotTokenSource(ctx, gcpSaChain...)
	token, err := tokenSource.Token()
	if err != nil {
		return fmt.Errorf("failed to get token: %v", err)
	}

	// First, update the default/gcr-json-key Secret, which is the
	// source-of-truth when creating new chart namespaces.
	cfgData := map[string][]byte{".dockercfg": DockerCfgJSON(token.AccessToken)}
	if err := kubeutils.UpdateSecret(ctx, k8s, &corev1.Secret{
		ObjectMeta: metav1.ObjectMeta{
			Name:      SecretName,
			Namespace: "default",
			Labels: map[string]string{
				// The chart-assignment-controller looks for this label and copies
				// gcr-json-key to new namespaces, so that the pods can pull images.
				"cloudrobotics.com/copy-to-chart-namespaces": "true",
			},
		},
		Type: corev1.SecretTypeDockercfg,
		Data: cfgData,
	}); err != nil {
		return fmt.Errorf("failed to update default/%s: %v", SecretName, err)
	}
	// Tell k8s to use this key by pointing the default SA at it.
	patchData := []byte(`{"imagePullSecrets": [{"name": "` + SecretName + `"}]}`)
	if err := patchServiceAccount(ctx, k8s, "default", "default", patchData); err != nil {
		return fmt.Errorf("failed to update kubernetes service account for namespace default: %v", err)
	}

	nsList, err := k8s.CoreV1().Namespaces().List(ctx, metav1.ListOptions{})
	if err != nil {
		return fmt.Errorf("failed to list namespaces: %v", err)
	}
	haveError := false
	for _, ns := range nsList.Items {
		if ns.DeletionTimestamp != nil {
			slog.Info("namespace is marked for deletion, skipping", slog.String("Namespace", ns.ObjectMeta.Name))
			continue
		}
		namespace := ns.ObjectMeta.Name
		if namespace == "default" {
			// Handled above.
			continue
		}

		// Only ever create secrets in a few specific, well-known namespaces. For app-* namespaces
		// the ChartAssignment controller will create the initial secret and patch the service account.
		// This avoids us putting pull secrets into eg foreign namespaces.
		s := k8s.CoreV1().Secrets(namespace)
		if _, err := s.Get(ctx, SecretName, metav1.GetOptions{}); k8serrors.IsNotFound(err) {
			if namespace != "kube-system" {
				continue
			}
		}
		// If we get here, the namespace has a secret that we need to update or
		// it is the kube-system namespace where it is okay to create the secret.

		// Create or update a secret containing a docker config with the access-token.
		err = kubeutils.UpdateSecret(ctx, k8s, &corev1.Secret{
			ObjectMeta: metav1.ObjectMeta{
				Name:      SecretName,
				Namespace: namespace,
			},
			Type: corev1.SecretTypeDockercfg,
			Data: cfgData,
		})
		if err != nil {
			slog.Error("failed to update kubernetes secret",
				slog.String("Namespace", namespace),
				ilog.Err(err))
			haveError = true
			continue
		}
		// Tell k8s to use this key by pointing the default SA at it.
		err = patchServiceAccount(ctx, k8s, "default", namespace, patchData)
		if err != nil {
			slog.Error("failed to update kubernetes service account",
				slog.String("Namespace", namespace),
				ilog.Err(err))
			haveError = true
		}
	}
	if haveError {
		return fmt.Errorf("failed to update one or more namespaces")
	}
	return nil
}
