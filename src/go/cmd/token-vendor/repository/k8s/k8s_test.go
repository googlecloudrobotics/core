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
	"errors"
	"testing"
	"time"

	corev1 "k8s.io/api/core/v1"
	kerrors "k8s.io/apimachinery/pkg/api/errors"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/apimachinery/pkg/util/wait"
	"k8s.io/client-go/kubernetes/fake"

	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/repository"
	registryv1alpha1 "github.com/googlecloudrobotics/core/src/go/pkg/apis/registry/v1alpha1"
	crfake "github.com/googlecloudrobotics/core/src/go/pkg/client/versioned/fake"
)

// Publish a key, retrieve it again and check listing of all keys.
func TestPublishListLookup(t *testing.T) {
	ctx := t.Context()
	cs := fake.NewSimpleClientset()
	kcl, err := NewK8sRepository(ctx, cs, crfake.NewSimpleClientset(), "default", "")
	if err != nil {
		t.Fatal(err)
	}
	const id = "testdevice"
	const key = "testkey"
	if err = kcl.PublishKey(ctx, id, key, repository.PublishOptions{}); err != nil {
		t.Fatal(err)
	}
	if _, err = kcl.LookupKey(ctx, id); err != nil {
		t.Fatal(err)
	}
	devices, err := kcl.ListAllDeviceIDs(ctx)
	if err != nil {
		t.Fatal(err)
	}
	if len(devices) != 1 || devices[0] != id {
		t.Fatalf(`ListAllDeviceIDs() = %v, want [%q]`, devices, id)
	}
}

// Publish a key and override it with another one.
func TestPublishKeyUpdate(t *testing.T) {
	ctx := t.Context()
	cs := fake.NewSimpleClientset()
	kcl, err := NewK8sRepository(ctx, cs, crfake.NewSimpleClientset(), "default", "")
	if err != nil {
		t.Fatal(err)
	}
	const id = "testdevice"
	const key2 = "testkey2"
	if err = kcl.PublishKey(ctx, id, "testkey", repository.PublishOptions{}); err != nil {
		t.Fatal(err)
	}
	if err = kcl.PublishKey(ctx, id, key2, repository.PublishOptions{}); err != nil {
		t.Fatal(err)
	}
	k, err := kcl.LookupKey(ctx, id)
	if err != nil {
		t.Fatal(err)
	}
	if k.PublicKey != key2 {
		t.Fatalf("LookupKey(..) = %q, want %q", k.PublicKey, key2)
	}
}

func TestLookupDoesNotExist(t *testing.T) {
	ctx := t.Context()
	cs := fake.NewSimpleClientset()
	kcl, err := NewK8sRepository(ctx, cs, crfake.NewSimpleClientset(), "default", "")
	if err != nil {
		t.Fatal(err)
	}
	k, err := kcl.LookupKey(ctx, "testdevice")
	if !errors.Is(err, repository.ErrNotFound) {
		t.Fatalf("LookupKey produced wrong error: got %v, want %v", err, repository.ErrNotFound)
	}
	if k != nil {
		t.Fatalf("LookupKey(..) = %q, want nil", k)
	}
}

func TestConfigure(t *testing.T) {
	ctx := t.Context()
	cs := fake.NewSimpleClientset()
	kcl, err := NewK8sRepository(ctx, cs, crfake.NewSimpleClientset(), "default", "")
	if err != nil {
		t.Fatal(err)
	}
	const id = "testdevice"
	const key = "testkey"
	if err = kcl.PublishKey(ctx, id, key, repository.PublishOptions{}); err != nil {
		t.Fatal(err)
	}
	opts := repository.KeyOptions{"svc@example.com", ""}
	if err := kcl.ConfigureKey(ctx, id, opts); err != nil {
		t.Fatal(err)
	}
	k, err := kcl.LookupKey(ctx, id)
	if err != nil {
		t.Fatal(err)
	}
	if k.SAName != "svc@example.com" {
		t.Fatalf("LookupKey: got %q, expected %q", k.SAName, "svc@example.com")
	}
}

func TestReConfigure(t *testing.T) {
	ctx := t.Context()
	cs := fake.NewSimpleClientset()
	kcl, err := NewK8sRepository(ctx, cs, crfake.NewSimpleClientset(), "default", "")
	if err != nil {
		t.Fatal(err)
	}
	const id = "testdevice"
	const key = "testkey"
	if err = kcl.PublishKey(ctx, id, key, repository.PublishOptions{}); err != nil {
		t.Fatal(err)
	}
	opts := repository.KeyOptions{"svc@example.com", ""}
	if err := kcl.ConfigureKey(ctx, id, opts); err != nil {
		t.Fatal(err)
	}
	// remove the config again
	opts = repository.KeyOptions{"", ""}
	if err := kcl.ConfigureKey(ctx, id, opts); err != nil {
		t.Fatal(err)
	}
	k, err := kcl.LookupKey(ctx, id)
	if err != nil {
		t.Fatal(err)
	}
	if k.SAName != "" {
		t.Fatalf("LookupKey: got %q, expected %q", k.SAName, "svc@example.com")
	}
}

func TestPublishKeySetsOwnerReferenceForMatchingRobotCR(t *testing.T) {
	ctx := t.Context()
	cs := fake.NewSimpleClientset()
	robot := &registryv1alpha1.Robot{
		ObjectMeta: metav1.ObjectMeta{
			Name:      "foo",
			Namespace: "default",
			UID:       types.UID("robot-foo-uid"),
		},
	}
	crcs := crfake.NewSimpleClientset(robot)
	kcl, err := NewK8sRepository(ctx, cs, crcs, "default", "")
	if err != nil {
		t.Fatal(err)
	}

	const matchedDeviceID = "robot-foo"
	const unmatchedDeviceID = "robot-unmatched"

	if err := kcl.PublishKey(ctx, matchedDeviceID, "testkey-foo", repository.PublishOptions{}); err != nil {
		t.Fatal(err)
	}
	if err := kcl.PublishKey(ctx, unmatchedDeviceID, "testkey-unmatched", repository.PublishOptions{}); err != nil {
		t.Fatal(err)
	}

	cm, err := cs.CoreV1().ConfigMaps("default").Get(ctx, matchedDeviceID, metav1.GetOptions{})
	if err != nil {
		t.Fatal(err)
	}
	if len(cm.OwnerReferences) != 1 {
		t.Fatalf("matched ConfigMap OwnerReferences = %v, want 1 entry", cm.OwnerReferences)
	}
	if got := cm.OwnerReferences[0]; got.Name != "foo" || got.UID != "robot-foo-uid" || got.Kind != "Robot" {
		t.Fatalf("matched ConfigMap OwnerReference = %+v, want Name=foo UID=robot-foo-uid Kind=Robot", got)
	}

	unmatchedCM, err := cs.CoreV1().ConfigMaps("default").Get(ctx, unmatchedDeviceID, metav1.GetOptions{})
	if err != nil {
		t.Fatal(err)
	}
	if len(unmatchedCM.OwnerReferences) != 0 {
		t.Fatalf("unmatched ConfigMap OwnerReferences = %v, want empty", unmatchedCM.OwnerReferences)
	}
}

func TestRobotCRCreatedAfterPublishKeySetsOwnerReference(t *testing.T) {
	ctx := t.Context()
	cs := fake.NewSimpleClientset()
	crcs := crfake.NewSimpleClientset()
	kcl, err := NewK8sRepository(ctx, cs, crcs, "default", "")
	if err != nil {
		t.Fatal(err)
	}

	const deviceID = "robot-bar"
	if err := kcl.PublishKey(ctx, deviceID, "testkey-bar", repository.PublishOptions{}); err != nil {
		t.Fatal(err)
	}

	robot := &registryv1alpha1.Robot{
		ObjectMeta: metav1.ObjectMeta{
			Name:      "bar",
			Namespace: "default",
			UID:       types.UID("robot-bar-uid"),
		},
	}
	if _, err := crcs.RegistryV1alpha1().Robots("default").Create(ctx, robot, metav1.CreateOptions{}); err != nil {
		t.Fatal(err)
	}

	err = wait.PollUntilContextTimeout(ctx, time.Millisecond, 5*time.Second, true, func(ctx context.Context) (bool, error) {
		cm, err := cs.CoreV1().ConfigMaps("default").Get(ctx, deviceID, metav1.GetOptions{})
		if err != nil {
			return false, err
		}
		return len(cm.OwnerReferences) == 1 && cm.OwnerReferences[0].UID == "robot-bar-uid", nil
	})
	if err != nil {
		t.Fatalf("expected ConfigMap %q to receive OwnerReference after Robot CR creation: %v", deviceID, err)
	}
}

// A key can be owned by a Robot CR that doesn't match its device ID. For
// example, a user might set up a new device as lab-pc-01, but the device
// authenticates as robot-node-1234 until its configuration tells it that name.
func TestPublishKeyWithRobotNameSetsOwnerReference(t *testing.T) {
	ctx := t.Context()
	cs := fake.NewSimpleClientset()
	crcs := crfake.NewSimpleClientset(
		// Matches the device ID, but mustn't own the key, as robot-name takes
		// precedence.
		&registryv1alpha1.Robot{
			ObjectMeta: metav1.ObjectMeta{
				Name:      "node-1234",
				Namespace: "default",
				UID:       types.UID("node-1234-uid"),
			},
		},
		&registryv1alpha1.Robot{
			ObjectMeta: metav1.ObjectMeta{
				Name:      "my-cluster",
				Namespace: "default",
				UID:       types.UID("my-cluster-uid"),
			},
		},
	)
	kcl, err := NewK8sRepository(ctx, cs, crcs, "default", "")
	if err != nil {
		t.Fatal(err)
	}

	const deviceID = "robot-node-1234"
	opts := repository.PublishOptions{RobotName: "my-cluster"}
	// Publish twice to cover both creating and updating the ConfigMap.
	for _, key := range []string{"testkey", "testkey2"} {
		if err := kcl.PublishKey(ctx, deviceID, key, opts); err != nil {
			t.Fatal(err)
		}

		cm, err := cs.CoreV1().ConfigMaps("default").Get(ctx, deviceID, metav1.GetOptions{})
		if err != nil {
			t.Fatal(err)
		}
		if len(cm.OwnerReferences) != 1 || cm.OwnerReferences[0].Name != "my-cluster" || cm.OwnerReferences[0].UID != "my-cluster-uid" {
			t.Errorf("after publishing %q: ConfigMap OwnerReferences = %+v, want Name=my-cluster UID=my-cluster-uid", key, cm.OwnerReferences)
		}
		if got := cm.Labels[robotNameLabel]; got != "my-cluster" {
			t.Errorf("after publishing %q: ConfigMap label %s = %q, want %q", key, robotNameLabel, got, "my-cluster")
		}
		if got := cm.Data[pubKey]; got != key {
			t.Errorf("ConfigMap key = %q, want %q", got, key)
		}
	}
}

func TestRobotCRCreatedAfterPublishKeyWithRobotNameSetsOwnerReference(t *testing.T) {
	ctx := t.Context()
	cs := fake.NewSimpleClientset()
	crcs := crfake.NewSimpleClientset()
	kcl, err := NewK8sRepository(ctx, cs, crcs, "default", "")
	if err != nil {
		t.Fatal(err)
	}

	const deviceID = "robot-node-5678"
	if err := kcl.PublishKey(ctx, deviceID, "testkey", repository.PublishOptions{RobotName: "later-cluster"}); err != nil {
		t.Fatal(err)
	}

	// The Robot CR matching the device ID is created first, but must not become
	// the owner, as the key names its owner explicitly.
	for _, robot := range []*registryv1alpha1.Robot{
		{
			ObjectMeta: metav1.ObjectMeta{
				Name:      "node-5678",
				Namespace: "default",
				UID:       types.UID("node-5678-uid"),
			},
		},
		{
			ObjectMeta: metav1.ObjectMeta{
				Name:      "later-cluster",
				Namespace: "default",
				UID:       types.UID("later-cluster-uid"),
			},
		},
	} {
		if _, err := crcs.RegistryV1alpha1().Robots("default").Create(ctx, robot, metav1.CreateOptions{}); err != nil {
			t.Fatal(err)
		}
	}

	var ownerRefs []metav1.OwnerReference
	err = wait.PollUntilContextTimeout(ctx, time.Millisecond, 5*time.Second, true, func(ctx context.Context) (bool, error) {
		cm, err := cs.CoreV1().ConfigMaps("default").Get(ctx, deviceID, metav1.GetOptions{})
		if err != nil {
			return false, err
		}
		ownerRefs = cm.OwnerReferences
		return len(ownerRefs) > 0, nil
	})
	if err != nil {
		t.Fatalf("expected ConfigMap %q to receive OwnerReference after Robot CR creation: %v", deviceID, err)
	}
	if len(ownerRefs) != 1 || ownerRefs[0].UID != "later-cluster-uid" {
		t.Errorf("ConfigMap OwnerReferences = %+v, want UID=later-cluster-uid", ownerRefs)
	}
}

func TestMigrateConfigMapsFromLegacyNamespace(t *testing.T) {
	ctx := t.Context()
	legacyRobotCM := &corev1.ConfigMap{
		ObjectMeta: metav1.ObjectMeta{
			Name:      "robot-migrated",
			Namespace: "app-token-vendor",
			Labels: map[string]string{
				managedByLabel: managedByValue,
			},
			Annotations: map[string]string{
				serviceAccountAnnotation: "svc@example.com",
			},
		},
		Data: map[string]string{pubKey: "migrated-key"},
	}
	staleLegacyCM := &corev1.ConfigMap{
		ObjectMeta: metav1.ObjectMeta{
			Name:      "robot-existing",
			Namespace: "app-token-vendor",
			Labels: map[string]string{
				managedByLabel: managedByValue,
			},
		},
		Data: map[string]string{pubKey: "old-backup-key"},
	}
	existingDefaultCM := &corev1.ConfigMap{
		ObjectMeta: metav1.ObjectMeta{
			Name:      "robot-existing",
			Namespace: "default",
			Labels: map[string]string{
				managedByLabel: managedByValue,
			},
		},
		Data: map[string]string{pubKey: "current-newer-key"},
	}
	unrelatedCM := &corev1.ConfigMap{
		ObjectMeta: metav1.ObjectMeta{
			Name:      "kube-root-ca.crt",
			Namespace: "app-token-vendor",
		},
		Data: map[string]string{"ca.crt": "cert-data"},
	}
	cs := fake.NewSimpleClientset(legacyRobotCM, staleLegacyCM, existingDefaultCM, unrelatedCM)
	robot := &registryv1alpha1.Robot{
		ObjectMeta: metav1.ObjectMeta{
			Name:      "migrated",
			Namespace: "default",
			UID:       types.UID("robot-migrated-uid"),
		},
	}
	crcs := crfake.NewSimpleClientset(robot)

	kcl, err := NewK8sRepository(ctx, cs, crcs, "default", "app-token-vendor")
	if err != nil {
		t.Fatal(err)
	}

	// Verify the ConfigMap was migrated to "default" and can be looked up
	key, err := kcl.LookupKey(ctx, "robot-migrated")
	if err != nil {
		t.Fatalf("LookupKey for migrated configmap failed: %v", err)
	}
	if key.PublicKey != "migrated-key" || key.SAName != "svc@example.com" {
		t.Fatalf("LookupKey = %+v, want PublicKey=migrated-key SAName=svc@example.com", key)
	}

	// Verify OwnerReference was added during migration
	migratedCM, err := cs.CoreV1().ConfigMaps("default").Get(ctx, "robot-migrated", metav1.GetOptions{})
	if err != nil {
		t.Fatal(err)
	}
	if len(migratedCM.OwnerReferences) != 1 || migratedCM.OwnerReferences[0].UID != "robot-migrated-uid" {
		t.Fatalf("migrated ConfigMap OwnerReferences = %+v, want UID=robot-migrated-uid", migratedCM.OwnerReferences)
	}

	// Verify the old ConfigMap was deleted from "app-token-vendor"
	if _, err := cs.CoreV1().ConfigMaps("app-token-vendor").Get(ctx, "robot-migrated", metav1.GetOptions{}); !kerrors.IsNotFound(err) {
		t.Fatalf("expected legacy ConfigMap to be deleted from app-token-vendor, got err: %v", err)
	}

	// Verify pre-existing ConfigMap in "default" was NOT overwritten by the stale backup in "app-token-vendor",
	// and that the stale copy in "app-token-vendor" was deleted.
	existingKey, err := kcl.LookupKey(ctx, "robot-existing")
	if err != nil {
		t.Fatalf("LookupKey for robot-existing failed: %v", err)
	}
	if existingKey.PublicKey != "current-newer-key" {
		t.Fatalf("existing ConfigMap in default was overwritten: got %q, want %q", existingKey.PublicKey, "current-newer-key")
	}
	if _, err := cs.CoreV1().ConfigMaps("app-token-vendor").Get(ctx, "robot-existing", metav1.GetOptions{}); !kerrors.IsNotFound(err) {
		t.Fatalf("expected stale ConfigMap robot-existing to be deleted from app-token-vendor, got err: %v", err)
	}

	// Verify unrelated ConfigMap in "app-token-vendor" was not touched
	if _, err := cs.CoreV1().ConfigMaps("app-token-vendor").Get(ctx, "kube-root-ca.crt", metav1.GetOptions{}); err != nil {
		t.Fatalf("expected unrelated ConfigMap kube-root-ca.crt to remain in app-token-vendor: %v", err)
	}
}
