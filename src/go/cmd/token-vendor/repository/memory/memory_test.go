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
	"testing"
)

// Test publish and lookup key
func TestMemoryBackend(t *testing.T) {
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
	if err != nil {
		t.Fatal(err)
	}
	if k != nil {
		t.Fatalf("LookupKey: got %q, expected empty response", k)
	}
}
