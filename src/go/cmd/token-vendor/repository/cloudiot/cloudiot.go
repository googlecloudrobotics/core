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

import (
	"context"
	"fmt"
	"log"
	"net/http"
	"time"

	"github.com/pkg/errors"
	iot "google.golang.org/api/cloudiot/v1"
	"google.golang.org/api/option"
)

// Registry defines the Cloud IoT device registry to use as key repository.
type Registry struct {
	Project  string
	Region   string
	Registry string
}

// CloudIoTRepository provides acccess to the Cloud IoT-backed public key registry.
type CloudIoTRepository struct {
	service *iot.Service
	r       Registry
}

// NewCloudIoTRepository creates a new Cloud IoT repostory client.
//
// `client` parameter is optional
func NewCloudIoTRepository(ctx context.Context, r Registry, client *http.Client) (*CloudIoTRepository, error) {
	service, err := iot.NewService(ctx, option.WithHTTPClient(client))
	if err != nil {
		return nil, errors.Wrap(err, "failed to created new CloudIoTRepository client")
	}
	repo := &CloudIoTRepository{r: r, service: service}
	return repo, nil
}

// PublishKey publishes a public key to the device registry.
func (c *CloudIoTRepository) PublishKey(ctx context.Context, deviceID, publicKey string) error {
	panic("not implemented yet")
}

// LookupKey retrieves the public key of a device from the IoT registry.
//
// An empty string return indicates that no key exists for the given identifier. If multiple
// keys are found only the first is returned.
func (c *CloudIoTRepository) LookupKey(ctx context.Context, deviceID string) (string, error) {
	if c.service == nil {
		return "", fmt.Errorf("IoT client not initialized")
	}
	path := c.devicePath(deviceID)
	device, err := c.service.Projects.Locations.Registries.Devices.
		Get(path).Context(ctx).FieldMask("credentials,blocked").Do()
	if err != nil {
		return "", errors.Wrap(err, "key lookup failed")
	}
	if device.Blocked {
		return "", fmt.Errorf("device %q is blocked, not returning keys", path)
	}
	keys := extractValidKeys(device)
	if len(keys) == 0 {
		return "", nil
	}
	// only return the first key if multiple are found
	// the API does not support multiple keys anyway
	if len(keys) > 1 {
		log.Printf("multiple keys found for device %q, only returning first", path)
	}
	return keys[0], nil
}

// devicePath constructs the absolute path to the IoT registry as used in the GCP API.
func (c *CloudIoTRepository) devicePath(deviceId string) string {
	return fmt.Sprintf("%s/devices/%s", c.registryPath(), deviceId)
}

// registryPath constructs the absolute path to the IoT registry as used in the GCP API.
func (c *CloudIoTRepository) registryPath() string {
	return fmt.Sprintf("projects/%s/locations/%s/registries/%s",
		c.r.Project, c.r.Region, c.r.Registry)
}

// extractValidKeys returns all valid device public keys of the device.
//
// Expiration is the only criteria considered at the moment.
func extractValidKeys(device *iot.Device) (publicKeys []string) {
	publicKeys = make([]string, 0, 1)
	for _, cred := range device.Credentials {
		if cred.PublicKey == nil {
			continue
		}
		expired, err := isExpired(cred)
		if err != nil {
			log.Printf("%s, ignoring key", err.Error())
			continue
		}
		if expired {
			log.Printf("ignoring expired key")
			continue
		}
		publicKeys = append(publicKeys, cred.PublicKey.Key)
	}
	return
}

// isExpired checks if a device credential is expired.
//
// Returns only false if it is not expired and
// no errors happenend during parsing.
func isExpired(cred *iot.DeviceCredential) (bool, error) {
	// the API returns this date string when no expiration is set.
	// this is NOT mentioned in the docs here:
	// https://cloud.google.com/iot/docs/reference/cloudiot/rest/v1/projects.locations.registries.devices#devicecredential
	if cred.ExpirationTime == "1970-01-01T00:00:00Z" {
		return false, nil
	}
	exp, err := time.Parse(time.RFC3339Nano, cred.ExpirationTime)
	if err != nil {
		return true, errors.Wrapf(err, "failed to parse expiration time %s", cred.ExpirationTime)
	}
	now := time.Now()
	if exp.Before(now) || exp.Equal(now) {
		return true, nil
	}
	return false, nil
}
