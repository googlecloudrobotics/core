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

package cloudiot

import "context"

type CloudIoTRepository struct {
	project  string
	region   string
	registry string
}

func NewCloudIoTRepository(ctx context.Context, project, region, registry string) (*CloudIoTRepository, error) {
	return &CloudIoTRepository{
			project:  project,
			region:   region,
			registry: registry},
		nil
}

// Publish a public key to the device registry.
func (iotr *CloudIoTRepository) PublishKey(ctx context.Context, deviceID, publicKey string) error {
	panic("not implemented yet")
}

// Lookup all public keys of a device from the IoT registry.
func (iotr *CloudIoTRepository) LookupKey(ctx context.Context, deviceID string) (publicKey string, err error) {
	panic("not implemented yet")
}
