// Copyright 2024 The Cloud Robotics Authors
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

// Package repository defines the api for the pub key stores
package repository

import (
	"context"
)

// Key holds data + metadata of a public key entry
type Key struct {
	// PublicKey contains the public key data
	PublicKey string
	// SAName is the optional GCP IAM service-account that has been associated.
	SAName string
}

// PubKeyRepository defines the api for the pub key stores
type PubKeyRepository interface {
	// LookupKey retrieves the public key of a device from the repository.
	// An empty string return indicates that no key exists for the given identifier or
	// that the device is blocked.
	LookupKey(ctx context.Context, deviceID string) (*Key, error)
	PublishKey(ctx context.Context, deviceID, publicKey string) error
}
