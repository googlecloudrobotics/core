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

package memory

import (
	"context"
	"errors"
	"testing"

	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/repository"
)

// Test publish and lookup key
func TestMemoryPublishAndLookup(t *testing.T) {
	m, err := NewMemoryRepository(context.TODO())
	if err != nil {
		t.Fatal(err)
	}
	if err := m.PublishKey(context.TODO(), "a", "akey"); err != nil {
		t.Fatal(err)
	}
	if err := m.PublishKey(context.TODO(), "b", "bkey"); err != nil {
		t.Fatal(err)
	}
	k, err := m.LookupKey(context.TODO(), "a")
	if err != nil {
		t.Fatal(err)
	}
	if k.PublicKey != "akey" {
		t.Fatalf("Key for a: got %q, want %q", k, "akey")
	}
	k, err = m.LookupKey(context.TODO(), "b")
	if err != nil {
		t.Fatal(err)
	}
	if k.PublicKey != "bkey" {
		t.Fatalf("Key for b: got %q, want %q", k, "bkey")
	}
}

func TestMemoryNotFound(t *testing.T) {
	m, err := NewMemoryRepository(context.TODO())
	if err != nil {
		t.Fatal(err)
	}
	k, err := m.LookupKey(context.TODO(), "a")
	if !errors.Is(err, repository.ErrNotFound) {
		t.Fatalf("LookupKey produced wrong error: got %v, want %v", err, repository.ErrNotFound)
	}
	if k != nil {
		t.Fatalf("LookupKey: got %q, expected empty response", k)
	}
}

// Test publish and lookup key
func TestMemoryConfigure(t *testing.T) {
	ctx := context.Background()
	m, err := NewMemoryRepository(ctx)
	if err != nil {
		t.Fatal(err)
	}
	if err := m.PublishKey(ctx, "a", "akey"); err != nil {
		t.Fatal(err)
	}
	opts := repository.KeyOptions{"svc@example.com", ""}
	if err := m.ConfigureKey(ctx, "a", opts); err != nil {
		t.Fatal(err)
	}
	k, err := m.LookupKey(ctx, "a")
	if err != nil {
		t.Fatal(err)
	}
	if k.SAName != "svc@example.com" {
		t.Fatalf("LookupKey: got %q, expected %q", k.SAName, "svc@example.com")
	}
}
