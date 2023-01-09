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

	log "github.com/sirupsen/logrus"
)

// MemoryRepository uses a in-memory datastructure to store the keys.
// Used only for integration tests.
type MemoryRepository struct {
	keys map[string]string
}

func NewMemoryRepository(ctx context.Context) (*MemoryRepository, error) {
	return &MemoryRepository{keys: map[string]string{}}, nil
}

func (m *MemoryRepository) PublishKey(ctx context.Context, deviceID, publicKey string) error {
	log.Debugf("Publishing public key for %q: %q", deviceID, publicKey)
	m.keys[deviceID] = publicKey
	return nil
}

func (m *MemoryRepository) LookupKey(ctx context.Context, deviceID string) (string, error) {
	log.Debugf("LookupKey for device %q", deviceID)
	// key not found does not need to be an error
	return m.keys[deviceID], nil
}
