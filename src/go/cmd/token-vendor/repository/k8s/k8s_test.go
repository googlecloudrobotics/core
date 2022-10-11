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
	"testing"

	"k8s.io/client-go/kubernetes/fake"
)

// Publish a key, retrieve it again and check listing of all keys.
func TestPublishListLookup(t *testing.T) {
	cs := fake.NewSimpleClientset()
	kcl, err := NewK8sRepository(context.TODO(), cs, "default")
	if err != nil {
		t.Fatal(err)
	}
	const id = "testdevice"
	const key = "testkey"
	if err = kcl.PublishKey(context.TODO(), id, key); err != nil {
		t.Fatal(err)
	}
	if _, err = kcl.LookupKey(context.TODO(), id); err != nil {
		t.Fatal(err)
	}
	devices, err := kcl.ListAllDeviceIDs(context.TODO())
	if err != nil {
		t.Fatal(err)
	}
	if len(devices) != 1 || devices[0] != id {
		t.Fatalf(`ListAllDeviceIDs() = %v, want [%q]`, devices, id)
	}
}

// Publish a key and override it with another one.
func TestPublishKeyUpdate(t *testing.T) {
	cs := fake.NewSimpleClientset()
	kcl, err := NewK8sRepository(context.TODO(), cs, "default")
	if err != nil {
		t.Fatal(err)
	}
	const id = "testdevice"
	const key2 = "testkey2"
	if err = kcl.PublishKey(context.TODO(), id, "testkey"); err != nil {
		t.Fatal(err)
	}
	if err = kcl.PublishKey(context.TODO(), id, key2); err != nil {
		t.Fatal(err)
	}
	k, err := kcl.LookupKey(context.TODO(), id)
	if err != nil {
		t.Fatal(err)
	}
	if k != key2 {
		t.Fatalf("LookupKey(..) = %q, want %q", k, key2)
	}
}

// Test if Lookup returns an empty key string in case the configmap is not found.
func TestLookupDoesNotExist(t *testing.T) {
	cs := fake.NewSimpleClientset()
	kcl, err := NewK8sRepository(context.TODO(), cs, "default")
	if err != nil {
		t.Fatal(err)
	}
	k, err := kcl.LookupKey(context.TODO(), "testdevice")
	if err != nil {
		t.Fatalf("LookupKey produced error %v, want nil", err)
	}
	if k != "" {
		t.Fatalf("LookupKey(..) = %q, want empty string", k)
	}
}
