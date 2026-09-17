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

	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
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
	kcl, err := NewK8sRepository(ctx, cs, crfake.NewSimpleClientset(), "default")
	if err != nil {
		t.Fatal(err)
	}
	const id = "testdevice"
	const key = "testkey"
	if err = kcl.PublishKey(ctx, id, key); err != nil {
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
	kcl, err := NewK8sRepository(ctx, cs, crfake.NewSimpleClientset(), "default")
	if err != nil {
		t.Fatal(err)
	}
	const id = "testdevice"
	const key2 = "testkey2"
	if err = kcl.PublishKey(ctx, id, "testkey"); err != nil {
		t.Fatal(err)
	}
	if err = kcl.PublishKey(ctx, id, key2); err != nil {
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
	kcl, err := NewK8sRepository(ctx, cs, crfake.NewSimpleClientset(), "default")
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
	kcl, err := NewK8sRepository(ctx, cs, crfake.NewSimpleClientset(), "default")
	if err != nil {
		t.Fatal(err)
	}
	const id = "testdevice"
	const key = "testkey"
	if err = kcl.PublishKey(ctx, id, key); err != nil {
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
	kcl, err := NewK8sRepository(ctx, cs, crfake.NewSimpleClientset(), "default")
	if err != nil {
		t.Fatal(err)
	}
	const id = "testdevice"
	const key = "testkey"
	if err = kcl.PublishKey(ctx, id, key); err != nil {
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

func TestRobotCRDeletionDeletesConfigMap(t *testing.T) {
	ctx := t.Context()
	cs := fake.NewSimpleClientset()
	robot := &registryv1alpha1.Robot{
		ObjectMeta: metav1.ObjectMeta{
			Name:      "foo",
			Namespace: "default",
		},
	}
	crcs := crfake.NewSimpleClientset(robot)
	kcl, err := NewK8sRepository(ctx, cs, crcs, "app-token-vendor")
	if err != nil {
		t.Fatal(err)
	}

	const matchedDeviceID = "robot-foo"
	const unmatchedDeviceID = "robot-unmatched"

	// Publish key for a device that has a matching Robot CR ("foo")
	if err := kcl.PublishKey(ctx, matchedDeviceID, "testkey-foo"); err != nil {
		t.Fatal(err)
	}
	// Publish key for a device that does NOT have a matching Robot CR
	if err := kcl.PublishKey(ctx, unmatchedDeviceID, "testkey-unmatched"); err != nil {
		t.Fatal(err)
	}

	cm, err := cs.CoreV1().ConfigMaps("app-token-vendor").Get(ctx, matchedDeviceID, metav1.GetOptions{})
	if err != nil {
		t.Fatal(err)
	}
	if got := cm.Labels[labelRobotName]; got != "foo" {
		t.Fatalf("matched ConfigMap label %q = %q, want %q", labelRobotName, got, "foo")
	}

	unmatchedCM, err := cs.CoreV1().ConfigMaps("app-token-vendor").Get(ctx, unmatchedDeviceID, metav1.GetOptions{})
	if err != nil {
		t.Fatal(err)
	}
	if got := unmatchedCM.Labels[labelRobotName]; got != "" {
		t.Fatalf("unmatched ConfigMap label %q = %q, want empty", labelRobotName, got)
	}

	// Delete the Robot CR in default namespace
	if err := crcs.RegistryV1alpha1().Robots("default").Delete(ctx, "foo", metav1.DeleteOptions{}); err != nil {
		t.Fatal(err)
	}

	// Wait for the ConfigMap to be automatically deleted
	err = wait.PollUntilContextTimeout(ctx, time.Millisecond, 5*time.Second, true, func(ctx context.Context) (bool, error) {
		_, lookupErr := kcl.LookupKey(ctx, matchedDeviceID)
		return errors.Is(lookupErr, repository.ErrNotFound), nil
	})
	if err != nil {
		t.Fatalf("expected ConfigMap %q to be deleted after Robot CR deletion: %v", matchedDeviceID, err)
	}

	// Unmatched ConfigMap should still exist
	if _, err := kcl.LookupKey(ctx, unmatchedDeviceID); err != nil {
		t.Fatalf("expected unmatched ConfigMap %q to still exist, got err: %v", unmatchedDeviceID, err)
	}
}

func TestRobotCRCreatedAfterPublishKey(t *testing.T) {
	ctx := t.Context()
	cs := fake.NewSimpleClientset()
	crcs := crfake.NewSimpleClientset()
	kcl, err := NewK8sRepository(ctx, cs, crcs, "app-token-vendor")
	if err != nil {
		t.Fatal(err)
	}

	const deviceID = "robot-bar"
	if err := kcl.PublishKey(ctx, deviceID, "testkey-bar"); err != nil {
		t.Fatal(err)
	}

	// Create the Robot CR after the key was published (e.g. setup-robot flow)
	robot := &registryv1alpha1.Robot{
		ObjectMeta: metav1.ObjectMeta{
			Name:      "bar",
			Namespace: "default",
		},
	}
	if _, err := crcs.RegistryV1alpha1().Robots("default").Create(ctx, robot, metav1.CreateOptions{}); err != nil {
		t.Fatal(err)
	}

	// Wait for the ConfigMap to receive the robot-name label
	err = wait.PollUntilContextTimeout(ctx, time.Millisecond, 5*time.Second, true, func(ctx context.Context) (bool, error) {
		cm, err := cs.CoreV1().ConfigMaps("app-token-vendor").Get(ctx, deviceID, metav1.GetOptions{})
		if err != nil {
			return false, err
		}
		return cm.Labels[labelRobotName] == "bar", nil
	})
	if err != nil {
		t.Fatalf("expected ConfigMap %q to be labeled with robot name: %v", deviceID, err)
	}

	// Delete the Robot CR and verify the ConfigMap is deleted
	if err := crcs.RegistryV1alpha1().Robots("default").Delete(ctx, "bar", metav1.DeleteOptions{}); err != nil {
		t.Fatal(err)
	}
	err = wait.PollUntilContextTimeout(ctx, time.Millisecond, 5*time.Second, true, func(ctx context.Context) (bool, error) {
		_, lookupErr := kcl.LookupKey(ctx, deviceID)
		return errors.Is(lookupErr, repository.ErrNotFound), nil
	})
	if err != nil {
		t.Fatalf("expected ConfigMap %q to be deleted after Robot CR deletion: %v", deviceID, err)
	}
}


