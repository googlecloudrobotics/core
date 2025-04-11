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
	"log/slog"

	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/repository"
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
	slog.Debug("PublishKey", slog.String("DeviceID", deviceID), slog.String("PublicKey", publicKey))
	m.keys[deviceID] = publicKey
	return nil
}

func (m *MemoryRepository) LookupKey(ctx context.Context, deviceID string) (*repository.Key, error) {
	slog.Debug("LookupKey", slog.String("DeviceID", deviceID))
	// key not found does not need to be an error
	k, found := m.keys[deviceID]
	if !found {
		return nil, repository.ErrNotFound
	}
	return &repository.Key{k, "", ""}, nil
}
