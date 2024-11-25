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

package k8s

import (
	"context"
	"fmt"
	"log/slog"

	"github.com/pkg/errors"
	corev1 "k8s.io/api/core/v1"
	kerrors "k8s.io/apimachinery/pkg/api/errors"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/client-go/kubernetes"

	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/repository"
)

// K8sRepository uses Kubernetes configmaps as public key backend for devices.
type K8sRepository struct {
	kcl kubernetes.Interface // client-go Clientset
	ns  string               // The namespace to use
}

// NewK8sRepository creates a new K8sRepository key repository.
//
// Use `ns` to specify an existing namespace to use for the device configmaps. Provide
// either a k8s.io/client-go/kubernetes/fake.NewSimpleClientset() for `kcl`
// for testing, or a real Interface from kubernetes.NewForConfig(..).
func NewK8sRepository(ctx context.Context, kcl kubernetes.Interface, ns string) (*K8sRepository, error) {
	return &K8sRepository{kcl: kcl, ns: ns}, nil
}

const (
	pubKey = "pubKey" // Configmap key for the public key
	// Configmap annotation specifies the service account to use (optional)
	serviceAccountAnnotation = "cloudrobotics.com/gcp-service-account"
)

// ListAllDeviceIDs returns a slice of all device identifiers found in the namespace.
func (k *K8sRepository) ListAllDeviceIDs(ctx context.Context) ([]string, error) {
	ls, err := k.kcl.CoreV1().ConfigMaps(k.ns).List(ctx, metav1.ListOptions{})
	if err != nil {
		return []string{}, errors.Wrapf(err, "failed to list configmaps from namespace %q", k.ns)
	}
	names := make([]string, 0)
	for _, cm := range ls.Items {
		names = append(names, cm.GetName())
	}
	return names, nil
}

// LookupKey returns the public key for a given device identifier.
//
// The public key is stored under a specific key in the configmap. If the configmap
// does not exist, we return an empty string. For any other error or a malformed
// configmap we return an error.
func (k *K8sRepository) LookupKey(ctx context.Context, deviceID string) (*repository.Key, error) {
	slog.Debug("looking up public key", slog.String("Namespace", k.ns), slog.String("ConfigMap", deviceID))
	cm, err := k.kcl.CoreV1().ConfigMaps(k.ns).Get(ctx, deviceID, metav1.GetOptions{})
	if kerrors.IsNotFound(err) {
		slog.Debug("ConfigMap not found", slog.String("Namespace", k.ns), slog.String("ConfigMap", deviceID))
		return nil, nil
	}
	if err != nil {
		return nil, errors.Wrapf(err, "failed to retrieve configmap %q/%q", k.ns, deviceID)
	}
	key, found := cm.Data[pubKey]
	if !found {
		return nil, fmt.Errorf("configmap %q/%q does not contain key %q", k.ns, deviceID, pubKey)
	}
	sa, _ := cm.ObjectMeta.Annotations[serviceAccountAnnotation]
	return &repository.Key{key, sa}, nil
}

// PublishKey sets or updates a public key for a given device identifier.
//
// If the configmap for a device does not exist yet it is created. If it exists
// already the public key section of the configmap is updated.
func (k *K8sRepository) PublishKey(ctx context.Context, deviceID, publicKey string) error {
	slog.Debug("publishing key", slog.String("DeviceID", deviceID))
	cm, err := createPubKeyDeviceConfig(deviceID, publicKey)
	if err != nil {
		return errors.Wrapf(err, "failed to init device configmap %q/%q", k.ns, deviceID)
	}
	if _, err = k.kcl.CoreV1().ConfigMaps(k.ns).Create(ctx, cm, metav1.CreateOptions{}); err == nil { // no error

		slog.Info("created new configmap", slog.String("Namespace", k.ns), slog.String("ConfigMap", deviceID))
		return nil
	}
	if !kerrors.IsAlreadyExists(err) { // any error not AlreadyExist
		return errors.Wrapf(err, "failed to create device configmap %q/%q", k.ns, deviceID)
	}
	// AlreadyExist error, updating configmap
	// We do not want to override any other keys besides the public key here.
	// createPubKeyDeviceConfig only creates a minimum configmap so updating is safe here.
	if _, err = k.kcl.CoreV1().ConfigMaps(k.ns).Update(ctx, cm, metav1.UpdateOptions{}); err != nil {
		return errors.Wrapf(err, "configmap %q/%q exists but failed to update it", k.ns, deviceID)
	}
	slog.Info("updated configmap with new public key", slog.String("Namespace", k.ns), slog.String("ConfigMap", deviceID))
	return nil
}

// createPubKeyDeviceConfig creates a configmap with only the public key in it.
//
// This is used also during update of existing devices. Make sure no default values
// are used here which could override a manually set key.
func createPubKeyDeviceConfig(name string, pk string) (*corev1.ConfigMap, error) {
	return &corev1.ConfigMap{
		TypeMeta: metav1.TypeMeta{
			Kind:       "ConfigMap",
			APIVersion: "v1",
		},
		ObjectMeta: metav1.ObjectMeta{
			Name: name,
			Labels: map[string]string{
				"app.kubernetes.io/managed-by": "token-vendor",
			},
		},
		Data: map[string]string{pubKey: pk},
	}, nil
}
