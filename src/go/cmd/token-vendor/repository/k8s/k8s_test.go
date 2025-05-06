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

	"k8s.io/client-go/kubernetes/fake"

	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/repository"
)

// Publish a key, retrieve it again and check listing of all keys.
func TestPublishListLookup(t *testing.T) {
	ctx := context.Background()
	cs := fake.NewSimpleClientset()
	kcl, err := NewK8sRepository(ctx, cs, "default")
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
	ctx := context.Background()
	cs := fake.NewSimpleClientset()
	kcl, err := NewK8sRepository(ctx, cs, "default")
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
	ctx := context.Background()
	cs := fake.NewSimpleClientset()
	kcl, err := NewK8sRepository(ctx, cs, "default")
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
	ctx := context.Background()
	cs := fake.NewSimpleClientset()
	kcl, err := NewK8sRepository(ctx, cs, "default")
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
