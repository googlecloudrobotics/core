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
	"time"

	"github.com/pkg/errors"

	corev1 "k8s.io/api/core/v1"
	kerrors "k8s.io/apimachinery/pkg/api/errors"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/watch"
	"k8s.io/client-go/kubernetes"
	"k8s.io/client-go/tools/cache"

	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/repository"
	"github.com/googlecloudrobotics/ilog"
)

// Re-list all ConfigMaps periodically in case we drop an event or otherwise
// have inconsistent state in the cache. May be unnecessary but provide some
// defence in depth.
const resyncPeriod = 5 * time.Minute

// K8sRepository uses Kubernetes configmaps as public key backend for devices.
type K8sRepository struct {
	kcl kubernetes.Interface // client-go Clientset
	ns  string               // The namespace to use

	cmInformer cache.SharedIndexInformer
}

// NewK8sRepository creates a new K8sRepository key repository.
//
// Use `ns` to specify an existing namespace to use for the device configmaps. Provide
// either a k8s.io/client-go/kubernetes/fake.NewSimpleClientset() for `kcl`
// for testing, or a real Interface from kubernetes.NewForConfig(..).
func NewK8sRepository(ctx context.Context, kcl kubernetes.Interface, ns string) (*K8sRepository, error) {
	// The informer provides an in-memory cache and prevents us from hammering the apiserver.
	cmInformer := cache.NewSharedIndexInformer(
		&cache.ListWatch{
			ListFunc: func(options metav1.ListOptions) (object runtime.Object, e error) {
				return kcl.CoreV1().ConfigMaps(ns).List(ctx, options)
			},
			WatchFunc: func(options metav1.ListOptions) (i watch.Interface, e error) {
				return kcl.CoreV1().ConfigMaps(ns).Watch(ctx, options)
			},
		},
		&corev1.ConfigMap{},
		resyncPeriod,
		cache.Indexers{},
	)
	go cmInformer.Run(ctx.Done())
	// Wait for the cache to sync before returning so we don't serve requests
	// until we're ready.
	if !cache.WaitForCacheSync(ctx.Done(), cmInformer.HasSynced) {
		return nil, fmt.Errorf("failed to sync configmap cache")
	}
	return &K8sRepository{kcl: kcl, ns: ns, cmInformer: cmInformer}, nil
}

const (
	pubKey = "pubKey" // Configmap key for the public key
	// Configmap annotation specifies the service account to use (optional)
	serviceAccountAnnotation = "cloudrobotics.com/gcp-service-account"
	// Configmap annotation specifies the intermediate service account delegate to use (optional)
	serviceAccountDelegateAnnotation = "cloudrobotics.com/gcp-service-account-delegate"
)

// ListAllDeviceIDs returns a slice of all device identifiers found in the namespace.
func (k *K8sRepository) ListAllDeviceIDs(ctx context.Context) ([]string, error) {
	objs := k.cmInformer.GetStore().List()
	names := make([]string, 0)
	for _, obj := range objs {
		cm, ok := obj.(*corev1.ConfigMap)
		if ok {
			names = append(names, cm.GetName())
		}
	}
	return names, nil
}

// LookupKey returns the public key for a given device identifier.
//
// The public key is stored under a specific key in the configmap. Returns an
// error if the configmap is not found or is not valid.
func (k *K8sRepository) LookupKey(ctx context.Context, deviceID string) (*repository.Key, error) {
	slog.Debug("looking up public key", slog.String("Namespace", k.ns), slog.String("ConfigMap", deviceID))
	obj, exists, err := k.cmInformer.GetStore().GetByKey(k.ns + "/" + deviceID)
	if err != nil {
		return nil, errors.Wrapf(err, "failed to retrieve configmap %q/%q from cache", k.ns, deviceID)
	}
	if !exists {
		return nil, errors.Wrapf(repository.ErrNotFound, "failed to retrieve configmap %q/%q", k.ns, deviceID)
	}
	cm, ok := obj.(*corev1.ConfigMap)
	if !ok {
		return nil, fmt.Errorf("unexpected object type: %T", obj)
	}

	key, found := cm.Data[pubKey]
	if !found {
		return nil, fmt.Errorf("configmap %q/%q does not contain key %q", k.ns, deviceID, pubKey)
	}
	sa := cm.ObjectMeta.Annotations[serviceAccountAnnotation]
	saDelegate := cm.ObjectMeta.Annotations[serviceAccountDelegateAnnotation]
	return &repository.Key{key, sa, saDelegate}, nil
}

// PublishKey sets or updates a public key for a given device identifier.
//
// If the configmap for a device does not exist yet it is created. If it exists
// already the public key section of the configmap is updated.
func (k *K8sRepository) PublishKey(ctx context.Context, deviceID, publicKey string) error {
	slog.Debug("publishing key", slog.String("DeviceID", deviceID))
	cm, err := createPubKeyDeviceConfig(deviceID, k.ns, publicKey)
	if err != nil {
		return errors.Wrapf(err, "failed to init device configmap %q/%q", k.ns, deviceID)
	}
	if _, err = k.kcl.CoreV1().ConfigMaps(k.ns).Create(ctx, cm, metav1.CreateOptions{}); err == nil { // no error
		k.cmInformer.GetStore().Add(cm)
		return nil
	}
	if !kerrors.IsAlreadyExists(err) { // any error not AlreadyExist
		return errors.Wrapf(err, "failed to create device configmap %q/%q", k.ns, deviceID)
	}
	// AlreadyExist error, updating configmap.
	// We do not want to override any other keys besides the public key here.
	// createPubKeyDeviceConfig only creates a minimum configmap so updating is safe here.
	if _, err = k.kcl.CoreV1().ConfigMaps(k.ns).Update(ctx, cm, metav1.UpdateOptions{}); err != nil {
		return errors.Wrapf(err, "configmap %q/%q exists but failed to update it", k.ns, deviceID)
	}
	// Update the informer store so that LookupKey can be used immediately.
	if err := k.cmInformer.GetStore().Update(cm); err != nil {
		slog.Warn("failed to update informer store", slog.String("DeviceID", deviceID), ilog.Err(err))
	}
	return nil
}

// createPubKeyDeviceConfig creates a configmap with only the public key in it.
//
// This is used also during update of existing devices. Make sure no default values
// are used here which could override a manually set key.
func createPubKeyDeviceConfig(name, namespace, pk string) (*corev1.ConfigMap, error) {
	return &corev1.ConfigMap{
		TypeMeta: metav1.TypeMeta{
			Kind:       "ConfigMap",
			APIVersion: "v1",
		},
		ObjectMeta: metav1.ObjectMeta{
			Namespace: namespace,
			Name:      name,
			Labels: map[string]string{
				"app.kubernetes.io/managed-by": "token-vendor",
			},
		},
		Data: map[string]string{pubKey: pk},
	}, nil
}
