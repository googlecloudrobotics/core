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
	"k8s.io/apimachinery/pkg/util/validation"
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
// Use `ns` to specify an existing namespace to use for the device configmaps, and
// `migrateFromNS` (optional) to specify a legacy namespace from which device configmaps
// should be migrated at startup. Provide either a k8s.io/client-go/kubernetes/fake.NewSimpleClientset()
// for `kcl` for testing, or a real Interface from kubernetes.NewForConfig(..).
func NewK8sRepository(ctx context.Context, kcl kubernetes.Interface, crcl versioned.Interface, ns, migrateFromNS string) (*K8sRepository, error) {
	repo := &K8sRepository{kcl: kcl, crcl: crcl, ns: ns}

	if migrateFromNS != "" && migrateFromNS != ns {
		if errs := validation.IsDNS1123Label(migrateFromNS); len(errs) > 0 {
			return nil, fmt.Errorf("invalid migrate-from-namespace %q: %s", migrateFromNS, strings.Join(errs, ", "))
		}
		if err := repo.migrateConfigMaps(ctx, migrateFromNS); err != nil {
			slog.WarnContext(ctx, "failed to migrate configmaps from legacy namespace",
				slog.String("FromNamespace", migrateFromNS), ilog.Err(err))
		}
	}

	// The informer provides an in-memory cache and prevents us from hammering the apiserver.
	cmInformer := cache.NewSharedIndexInformer(
		cache.ToListWatcherWithWatchListSemantics(&cache.ListWatch{
			ListFunc: func(options metav1.ListOptions) (object runtime.Object, e error) {
				options.LabelSelector = managedByLabel + "=" + managedByValue
				return kcl.CoreV1().ConfigMaps(ns).List(ctx, options)
			},
			WatchFunc: func(options metav1.ListOptions) (i watch.Interface, e error) {
				options.LabelSelector = managedByLabel + "=" + managedByValue
				return kcl.CoreV1().ConfigMaps(ns).Watch(ctx, options)
			},
		}, kcl),
		&corev1.ConfigMap{},
		resyncPeriod,
		cache.Indexers{robotNameIndex: indexByRobotName},
	)
	repo.cmInformer = cmInformer
	go cmInformer.Run(ctx.Done())
	// Wait for the cache to sync before returning so we don't serve requests
	// until we're ready. Use a 1ms poll interval instead of cache.WaitForCacheSync
	// (which hardcodes 100ms and slows down unit tests).
	if err := wait.PollUntilContextCancel(ctx, time.Millisecond, true, func(ctx context.Context) (bool, error) {
		return cmInformer.HasSynced(), nil
	}); err != nil {
		return nil, fmt.Errorf("failed to sync configmap cache: %w", err)
	}

	if crcl != nil {
		robotInformer := cache.NewSharedIndexInformer(
			cache.ToListWatcherWithWatchListSemantics(&cache.ListWatch{
				ListFunc: func(options metav1.ListOptions) (object runtime.Object, e error) {
					return crcl.RegistryV1alpha1().Robots(ns).List(ctx, options)
				},
				WatchFunc: func(options metav1.ListOptions) (i watch.Interface, e error) {
					return crcl.RegistryV1alpha1().Robots(ns).Watch(ctx, options)
				},
			}, kcl),
			&registryv1alpha1.Robot{},
			resyncPeriod,
			cache.Indexers{},
		)
		reg, err := robotInformer.AddEventHandler(cache.ResourceEventHandlerFuncs{
			AddFunc: func(obj any) {
				if robot, ok := obj.(*registryv1alpha1.Robot); ok {
					repo.onRobotAdded(ctx, robot)
				}
			},
		})
		if err != nil {
			return nil, fmt.Errorf("failed to add robot event handler: %w", err)
		}
		repo.robotInformer = robotInformer
		go robotInformer.Run(ctx.Done())
		if err := wait.PollUntilContextCancel(ctx, time.Millisecond, true, func(ctx context.Context) (bool, error) {
			return reg.HasSynced(), nil
		}); err != nil {
			return nil, fmt.Errorf("failed to sync robot cache: %w", err)
		}
	}

	return repo, nil
}

const (
	pubKey = "pubKey" // Configmap key for the public key
	// Configmap annotation specifies the service account to use (optional)
	serviceAccountAnnotation = "cloudrobotics.com/gcp-service-account"
	// Configmap annotation specifies the intermediate service account delegate to use (optional)
	serviceAccountDelegateAnnotation = "cloudrobotics.com/gcp-service-account-delegate"

	managedByLabel = "app.kubernetes.io/managed-by"
	managedByValue = "token-vendor"
	// Configmap label specifies the Robot CR that owns the key, if it was
	// named explicitly instead of being derived from the device ID (optional)
	robotNameLabel = "cloudrobotics.com/robot-name"
	robotPrefix    = "robot-"

	// Configmap informer index from the name of a Robot CR to the configmaps
	// it should own.
	robotNameIndex = "robotName"
)

func robotOwnerRef(robot *registryv1alpha1.Robot) *metav1.OwnerReference {
	return &metav1.OwnerReference{
		APIVersion:         registryv1alpha1.SchemeGroupVersion.String(),
		Kind:               "Robot",
		Name:               robot.GetName(),
		UID:                robot.GetUID(),
		BlockOwnerDeletion: new(true),
		Controller:         new(true),
	}
}

func setOwnerReference(om *metav1.ObjectMeta, ref *metav1.OwnerReference) bool {
	for i, or := range om.OwnerReferences {
		if or.Kind == ref.Kind && or.Name == ref.Name {
			if or.UID == ref.UID && or.APIVersion == ref.APIVersion {
				return false
			}
			om.OwnerReferences[i] = *ref
			return true
		}
	}
	om.OwnerReferences = append(om.OwnerReferences, *ref)
	return true
}

// robotNameFromDeviceID derives the name of the Robot CR from a device ID of the
// form "robot-<robot-name>". Returns "" if deviceID doesn't have that form.
func robotNameFromDeviceID(deviceID string) string {
	if !strings.HasPrefix(deviceID, robotPrefix) {
		return ""
	}
	return strings.TrimPrefix(deviceID, robotPrefix)
}

// robotNameForConfigMap returns the name of the Robot CR that should own cm:
// the name from the robot name label if set, or else the name derived from the
// device ID. Returns "" if neither applies.
func robotNameForConfigMap(cm *corev1.ConfigMap) string {
	if robotName := cm.Labels[robotNameLabel]; robotName != "" {
		return robotName
	}
	return robotNameFromDeviceID(cm.GetName())
}

// indexByRobotName is a cache.IndexFunc that indexes configmaps by the name of
// the Robot CR that should own them.
func indexByRobotName(obj any) ([]string, error) {
	cm, ok := obj.(*corev1.ConfigMap)
	if !ok {
		return nil, nil
	}
	if robotName := robotNameForConfigMap(cm); robotName != "" {
		return []string{robotName}, nil
	}
	return nil, nil
}

// matchingRobotOwnerRef checks if a Robot CR called robotName exists in the
// repository namespace. Returns an OwnerReference to the Robot CR if found, or
// nil otherwise.
func (k *K8sRepository) matchingRobotOwnerRef(ctx context.Context, robotName string) *metav1.OwnerReference {
	if k.crcl == nil || robotName == "" {
		return nil
	}
	// TODO(rodrigoq): The cache can be stale if the Robot CR was just deleted
	// and re-created, eg when the device-manager replaces a cluster. Then the
	// owner reference has the old Robot's UID, and the garbage collector may
	// delete the configmap before onRobotAdded fixes the owner. Consider always
	// doing a live GET instead.
	if k.robotInformer != nil {
		if obj, exists, err := k.robotInformer.GetStore().GetByKey(k.ns + "/" + robotName); err == nil && exists {
			if robot, ok := obj.(*registryv1alpha1.Robot); ok {
				return robotOwnerRef(robot)
			}
		}
	}
	robot, err := k.crcl.RegistryV1alpha1().Robots(k.ns).Get(ctx, robotName, metav1.GetOptions{})
	if err != nil {
		return nil
	}
	if k.robotInformer != nil {
		_ = k.robotInformer.GetStore().Add(robot)
	}
	return robotOwnerRef(robot)
}

// onRobotAdded sets an OwnerReference to robot on the configmaps it should own,
// in case their keys were published before the Robot CR was created.
func (k *K8sRepository) onRobotAdded(ctx context.Context, robot *registryv1alpha1.Robot) {
	objs, err := k.cmInformer.GetIndexer().ByIndex(robotNameIndex, robot.GetName())
	if err != nil {
		slog.WarnContext(ctx, "failed to look up configmaps for robot", slog.String("RobotName", robot.GetName()), ilog.Err(err))
		return
	}
	for _, obj := range objs {
		cm, ok := obj.(*corev1.ConfigMap)
		if !ok {
			continue
		}
		deviceID := cm.GetName()
		cmCopy := cm.DeepCopy()
		if !setOwnerReference(&cmCopy.ObjectMeta, robotOwnerRef(robot)) {
			continue
		}
		updated, err := k.kcl.CoreV1().ConfigMaps(k.ns).Update(ctx, cmCopy, metav1.UpdateOptions{})
		if err != nil {
			slog.WarnContext(ctx, "failed to set owner reference on configmap for robot", slog.String("DeviceID", deviceID), ilog.Err(err))
			continue
		}
		if err := k.cmInformer.GetStore().Update(updated); err != nil {
			slog.WarnContext(ctx, "failed to update informer store", slog.String("DeviceID", deviceID), ilog.Err(err))
		}
	}
}

// migrateConfigMaps moves device public key ConfigMaps from migrateFromNS
// into k.ns (e.g. "default") at startup, attaching OwnerReferences to any
// matching Robot CRs.
func (k *K8sRepository) migrateConfigMaps(ctx context.Context, migrateFromNS string) error {
	cms, err := k.kcl.CoreV1().ConfigMaps(migrateFromNS).List(ctx, metav1.ListOptions{
		LabelSelector: managedByLabel + "=" + managedByValue,
	})
	if err != nil {
		if kerrors.IsNotFound(err) {
			return nil
		}
		return fmt.Errorf("failed to list configmaps in %q: %w", migrateFromNS, err)
	}
	for _, oldCM := range cms.Items {
		slog.InfoContext(ctx, "migrating device public key configmap",
			slog.String("ConfigMap", oldCM.Name),
			slog.String("FromNamespace", migrateFromNS),
			slog.String("ToNamespace", k.ns))
		labels := make(map[string]string, len(oldCM.Labels)+1)
		for k, v := range oldCM.Labels {
			labels[k] = v
		}
		labels[managedByLabel] = managedByValue
		newCM := &corev1.ConfigMap{
			TypeMeta: metav1.TypeMeta{
				Kind:       "ConfigMap",
				APIVersion: "v1",
			},
			ObjectMeta: metav1.ObjectMeta{
				Name:        oldCM.Name,
				Namespace:   k.ns,
				Labels:      labels,
				Annotations: oldCM.Annotations,
			},
			Data: oldCM.Data,
		}
		if ownerRef := k.matchingRobotOwnerRef(ctx, robotNameForConfigMap(newCM)); ownerRef != nil {
			newCM.OwnerReferences = []metav1.OwnerReference{*ownerRef}
		}
		if _, err := k.kcl.CoreV1().ConfigMaps(k.ns).Create(ctx, newCM, metav1.CreateOptions{}); err != nil && !kerrors.IsAlreadyExists(err) {
			slog.WarnContext(ctx, "failed to create migrated configmap", slog.String("DeviceID", oldCM.Name), ilog.Err(err))
			continue
		}
		if err := k.kcl.CoreV1().ConfigMaps(migrateFromNS).Delete(ctx, oldCM.Name, metav1.DeleteOptions{}); err != nil && !kerrors.IsNotFound(err) {
			slog.WarnContext(ctx, "failed to delete legacy configmap after migration", slog.String("DeviceID", oldCM.Name), ilog.Err(err))
		}
	}
	return nil
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
//
// The configmap is owned by the Robot CR named by opts.RobotName (or else the
// one derived from the device ID), so that it's deleted along with the Robot.
// If the Robot CR doesn't exist yet, the owner is set once it's created.
func (k *K8sRepository) PublishKey(ctx context.Context, deviceID, publicKey string, opts repository.PublishOptions) error {
	slog.DebugContext(ctx, "publishing key", slog.String("DeviceID", deviceID), slog.String("RobotName", opts.RobotName))
	robotName := opts.RobotName
	if robotName == "" {
		robotName = robotNameFromDeviceID(deviceID)
	}
	ownerRef := k.matchingRobotOwnerRef(ctx, robotName)
	cm, err := createPubKeyDeviceConfig(deviceID, k.ns, publicKey, opts.RobotName, ownerRef)
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
//
// robotName is the explicitly requested owner (if any), which is stored in a
// label so that the owner reference can be set if the Robot CR is created later.
func createPubKeyDeviceConfig(name, namespace, pk, robotName string, ownerRef *metav1.OwnerReference) (*corev1.ConfigMap, error) {
	cm := &corev1.ConfigMap{
		TypeMeta: metav1.TypeMeta{
			Kind:       "ConfigMap",
			APIVersion: "v1",
		},
		ObjectMeta: metav1.ObjectMeta{
			Namespace: namespace,
			Name:      name,
			Labels: map[string]string{
				managedByLabel: managedByValue,
			},
		},
		Data: map[string]string{pubKey: pk},
	}
	if robotName != "" {
		cm.Labels[robotNameLabel] = robotName
	}
	if ownerRef != nil {
		cm.OwnerReferences = []metav1.OwnerReference{*ownerRef}
	}
	return cm, nil
}

func mapSetOrDelete(m map[string]string, k, v string) {
	if v != "" {
		m[k] = v
	} else {
		delete(m, k)
	}
}
