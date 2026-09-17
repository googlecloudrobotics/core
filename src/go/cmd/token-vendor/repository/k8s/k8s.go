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
	"strings"
	"time"

	corev1 "k8s.io/api/core/v1"
	kerrors "k8s.io/apimachinery/pkg/api/errors"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/util/wait"
	"k8s.io/apimachinery/pkg/watch"
	"k8s.io/client-go/kubernetes"
	"k8s.io/client-go/tools/cache"

	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/repository"
	registryv1alpha1 "github.com/googlecloudrobotics/core/src/go/pkg/apis/registry/v1alpha1"
	"github.com/googlecloudrobotics/core/src/go/pkg/client/versioned"
	"github.com/googlecloudrobotics/ilog"
)

// Re-list all ConfigMaps periodically. If this causes problems, consider
// setting to 0 instead to disable, but hopefully this provides provides
// some defense against bugs without being too costly.
const resyncPeriod = 1 * time.Hour

// K8sRepository uses Kubernetes configmaps as public key backend for devices.
type K8sRepository struct {
	kcl  kubernetes.Interface // client-go Clientset
	crcl versioned.Interface  // Cloud Robotics CRD Clientset
	ns   string               // The namespace to use

	cmInformer    cache.SharedIndexInformer
	robotInformer cache.SharedIndexInformer
}

// NewK8sRepository creates a new K8sRepository key repository.
//
// Use `ns` to specify an existing namespace to use for the device configmaps. Provide
// either a k8s.io/client-go/kubernetes/fake.NewSimpleClientset() for `kcl`
// for testing, or a real Interface from kubernetes.NewForConfig(..).
func NewK8sRepository(ctx context.Context, kcl kubernetes.Interface, crcl versioned.Interface, ns string) (*K8sRepository, error) {
	// The informer provides an in-memory cache and prevents us from hammering the apiserver.
	cmInformer := cache.NewSharedIndexInformer(
		cache.ToListWatcherWithWatchListSemantics(&cache.ListWatch{
			ListFunc: func(options metav1.ListOptions) (object runtime.Object, e error) {
				return kcl.CoreV1().ConfigMaps(ns).List(ctx, options)
			},
			WatchFunc: func(options metav1.ListOptions) (i watch.Interface, e error) {
				return kcl.CoreV1().ConfigMaps(ns).Watch(ctx, options)
			},
		}, kcl),
		&corev1.ConfigMap{},
		resyncPeriod,
		cache.Indexers{},
	)
	go cmInformer.Run(ctx.Done())
	// Wait for the cache to sync before returning so we don't serve requests
	// until we're ready. Use a 1ms poll interval instead of cache.WaitForCacheSync
	// (which hardcodes 100ms and slows down unit tests).
	if err := wait.PollUntilContextCancel(ctx, time.Millisecond, true, func(ctx context.Context) (bool, error) {
		return cmInformer.HasSynced(), nil
	}); err != nil {
		return nil, fmt.Errorf("failed to sync configmap cache: %w", err)
	}

	repo := &K8sRepository{kcl: kcl, crcl: crcl, ns: ns, cmInformer: cmInformer}

	if crcl != nil {
		robotInformer := cache.NewSharedIndexInformer(
			cache.ToListWatcherWithWatchListSemantics(&cache.ListWatch{
				ListFunc: func(options metav1.ListOptions) (object runtime.Object, e error) {
					return crcl.RegistryV1alpha1().Robots(robotNamespace).List(ctx, options)
				},
				WatchFunc: func(options metav1.ListOptions) (i watch.Interface, e error) {
					return crcl.RegistryV1alpha1().Robots(robotNamespace).Watch(ctx, options)
				},
			}, kcl),
			&registryv1alpha1.Robot{},
			resyncPeriod,
			cache.Indexers{},
		)
		if _, err := robotInformer.AddEventHandler(cache.ResourceEventHandlerFuncs{
			AddFunc: func(obj any) {
				if name := extractRobotName(obj); name != "" {
					repo.onRobotAdded(ctx, name)
				}
			},
			DeleteFunc: func(obj any) {
				if name := extractRobotName(obj); name != "" {
					repo.onRobotDeleted(ctx, name)
				}
			},
		}); err != nil {
			return nil, fmt.Errorf("failed to add robot event handler: %w", err)
		}
		repo.robotInformer = robotInformer
		go robotInformer.Run(ctx.Done())
		if err := wait.PollUntilContextCancel(ctx, time.Millisecond, true, func(ctx context.Context) (bool, error) {
			return robotInformer.HasSynced(), nil
		}); err != nil {
			return nil, fmt.Errorf("failed to sync robot cache: %w", err)
		}
		repo.cleanupOrphanedConfigMaps(ctx)
	}

	return repo, nil
}

const (
	pubKey = "pubKey" // Configmap key for the public key
	// Configmap annotation specifies the service account to use (optional)
	serviceAccountAnnotation = "cloudrobotics.com/gcp-service-account"
	// Configmap annotation specifies the intermediate service account delegate to use (optional)
	serviceAccountDelegateAnnotation = "cloudrobotics.com/gcp-service-account-delegate"
	// Configmap label linking the device key to its Robot CR in the default namespace
	labelRobotName = "cloudrobotics.com/robot-name"
	robotPrefix    = "robot-"
	robotNamespace = "default"
)

func extractRobotName(obj any) string {
	if r, ok := obj.(*registryv1alpha1.Robot); ok {
		return r.GetName()
	}
	if tombstone, ok := obj.(cache.DeletedFinalStateUnknown); ok {
		if r, ok := tombstone.Obj.(*registryv1alpha1.Robot); ok {
			return r.GetName()
		}
	}
	return ""
}

// matchingRobotName checks if deviceID has the "robot-" prefix and if a
// matching Robot CR exists in the default namespace. Returns the robot name
// if found, or "" otherwise.
func (k *K8sRepository) matchingRobotName(ctx context.Context, deviceID string) string {
	if k.crcl == nil || !strings.HasPrefix(deviceID, robotPrefix) {
		return ""
	}
	robotName := strings.TrimPrefix(deviceID, robotPrefix)
	if robotName == "" {
		return ""
	}
	if k.robotInformer != nil {
		if _, exists, err := k.robotInformer.GetStore().GetByKey(robotNamespace + "/" + robotName); err == nil && exists {
			return robotName
		}
	}
	robot, err := k.crcl.RegistryV1alpha1().Robots(robotNamespace).Get(ctx, robotName, metav1.GetOptions{})
	if err != nil {
		return ""
	}
	if k.robotInformer != nil {
		_ = k.robotInformer.GetStore().Add(robot)
	}
	return robotName
}

func (k *K8sRepository) onRobotAdded(ctx context.Context, robotName string) {
	deviceID := robotPrefix + robotName
	obj, exists, err := k.cmInformer.GetStore().GetByKey(k.ns + "/" + deviceID)
	if err != nil || !exists {
		return
	}
	cm, ok := obj.(*corev1.ConfigMap)
	if !ok || cm.Labels[labelRobotName] == robotName {
		return
	}
	cmCopy := cm.DeepCopy()
	if cmCopy.Labels == nil {
		cmCopy.Labels = make(map[string]string)
	}
	cmCopy.Labels[labelRobotName] = robotName
	updated, err := k.kcl.CoreV1().ConfigMaps(k.ns).Update(ctx, cmCopy, metav1.UpdateOptions{})
	if err != nil {
		slog.WarnContext(ctx, "failed to label configmap for added robot", slog.String("DeviceID", deviceID), ilog.Err(err))
		return
	}
	if err := k.cmInformer.GetStore().Update(updated); err != nil {
		slog.WarnContext(ctx, "failed to update informer store", slog.String("DeviceID", deviceID), ilog.Err(err))
	}
}

func (k *K8sRepository) onRobotDeleted(ctx context.Context, robotName string) {
	deviceID := robotPrefix + robotName
	obj, exists, err := k.cmInformer.GetStore().GetByKey(k.ns + "/" + deviceID)
	if err != nil || !exists {
		return
	}
	cm, ok := obj.(*corev1.ConfigMap)
	if !ok || cm.Labels[labelRobotName] != robotName {
		return
	}
	slog.InfoContext(ctx, "deleting public key configmap for deleted robot", slog.String("DeviceID", deviceID), slog.String("Robot", robotName))
	if err := k.kcl.CoreV1().ConfigMaps(k.ns).Delete(ctx, deviceID, metav1.DeleteOptions{}); err != nil && !kerrors.IsNotFound(err) {
		slog.WarnContext(ctx, "failed to delete configmap for deleted robot", slog.String("DeviceID", deviceID), ilog.Err(err))
		return
	}
	if err := k.cmInformer.GetStore().Delete(cm); err != nil {
		slog.WarnContext(ctx, "failed to delete configmap from informer store", slog.String("DeviceID", deviceID), ilog.Err(err))
	}
}

func (k *K8sRepository) cleanupOrphanedConfigMaps(ctx context.Context) {
	for _, obj := range k.cmInformer.GetStore().List() {
		cm, ok := obj.(*corev1.ConfigMap)
		if !ok {
			continue
		}
		robotName := cm.Labels[labelRobotName]
		if robotName == "" {
			continue
		}
		if _, exists, err := k.robotInformer.GetStore().GetByKey(robotNamespace + "/" + robotName); err == nil && !exists {
			k.onRobotDeleted(ctx, robotName)
		}
	}
}

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
	slog.DebugContext(ctx, "looking up public key", slog.String("Namespace", k.ns), slog.String("ConfigMap", deviceID))
	obj, exists, err := k.cmInformer.GetStore().GetByKey(k.ns + "/" + deviceID)
	if err != nil {
		return nil, fmt.Errorf("failed to retrieve configmap %q/%q from cache: %w", k.ns, deviceID, err)
	}
	if !exists {
		return nil, fmt.Errorf("failed to retrieve configmap %q/%q: %w", k.ns, deviceID, repository.ErrNotFound)
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
	slog.DebugContext(ctx, "publishing key", slog.String("DeviceID", deviceID))
	robotName := k.matchingRobotName(ctx, deviceID)
	cm, err := createPubKeyDeviceConfig(deviceID, k.ns, publicKey, robotName)
	if err != nil {
		return fmt.Errorf("failed to init device configmap %q/%q: %w", k.ns, deviceID, err)
	}
	_, err = k.kcl.CoreV1().ConfigMaps(k.ns).Create(ctx, cm, metav1.CreateOptions{})
	if err == nil { // no error
		// Add to the informer store so that LookupKey can be used immediately.
		if err := k.cmInformer.GetStore().Add(cm); err != nil {
			slog.WarnContext(ctx, "failed to add to informer store", slog.String("DeviceID", deviceID), ilog.Err(err))
		}
		return nil
	}
	if !kerrors.IsAlreadyExists(err) { // any error not AlreadyExist
		return fmt.Errorf("failed to create device configmap %q/%q: %w", k.ns, deviceID, err)
	}
	// AlreadyExist error, updating configmap.
	// We do not want to override any other keys besides the public key here.
	// createPubKeyDeviceConfig only creates a minimum configmap so updating is safe here.
	if _, err := k.kcl.CoreV1().ConfigMaps(k.ns).Update(ctx, cm, metav1.UpdateOptions{}); err != nil {
		return fmt.Errorf("configmap %q/%q exists but failed to update it: %w", k.ns, deviceID, err)
	}
	// Update the informer store so that LookupKey can be used immediately.
	if err := k.cmInformer.GetStore().Update(cm); err != nil {
		slog.WarnContext(ctx, "failed to update informer store", slog.String("DeviceID", deviceID), ilog.Err(err))
	}
	return nil
}

func (k *K8sRepository) ConfigureKey(ctx context.Context, deviceID string, opts repository.KeyOptions) error {
	cm, err := k.kcl.CoreV1().ConfigMaps(k.ns).Get(ctx, deviceID, metav1.GetOptions{})
	if err != nil {
		if kerrors.IsNotFound(err) {
			return fmt.Errorf("failed to retrieve configmap %q/%q: %w", k.ns, deviceID, repository.ErrNotFound)
		}
		return fmt.Errorf("failed to retrieve configmap %q/%q from cache: %w", k.ns, deviceID, err)
	}
	if cm.ObjectMeta.Annotations == nil {
		cm.ObjectMeta.Annotations = make(map[string]string)
	}
	mapSetOrDelete(cm.ObjectMeta.Annotations, serviceAccountAnnotation, opts.ServiceAccount)
	mapSetOrDelete(cm.ObjectMeta.Annotations, serviceAccountDelegateAnnotation, opts.ServiceAccountDelegate)
	if _, err := k.kcl.CoreV1().ConfigMaps(k.ns).Update(ctx, cm, metav1.UpdateOptions{}); err != nil {
		return fmt.Errorf("failed to update configmap %q/%q: %w", k.ns, deviceID, err)
	}
	// Update the informer store so that LookupKey can be used immediately.
	if err := k.cmInformer.GetStore().Update(cm); err != nil {
		slog.WarnContext(ctx, "failed to update informer store", slog.String("DeviceID", deviceID), ilog.Err(err))
	}
	return nil
}

// createPubKeyDeviceConfig creates a configmap with only the public key in it.
//
// This is used also during update of existing devices. Make sure no default values
// are used here which could override a manually set key.
func createPubKeyDeviceConfig(name, namespace, pk, robotName string) (*corev1.ConfigMap, error) {
	labels := map[string]string{
		"app.kubernetes.io/managed-by": "token-vendor",
	}
	if robotName != "" {
		labels[labelRobotName] = robotName
	}
	return &corev1.ConfigMap{
		TypeMeta: metav1.TypeMeta{
			Kind:       "ConfigMap",
			APIVersion: "v1",
		},
		ObjectMeta: metav1.ObjectMeta{
			Namespace: namespace,
			Name:      name,
			Labels:    labels,
		},
		Data: map[string]string{pubKey: pk},
	}, nil
}

func mapSetOrDelete(m map[string]string, k, v string) {
	if v != "" {
		m[k] = v
	} else {
		delete(m, k)
	}
}
