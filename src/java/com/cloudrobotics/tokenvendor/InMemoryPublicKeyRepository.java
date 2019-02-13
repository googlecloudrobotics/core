// Copyright 2019 The Cloud Robotics Authors
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

package com.cloudrobotics.tokenvendor;

import com.google.api.client.util.ArrayMap;
import com.google.common.collect.ImmutableList;
import java.util.Map;
import javax.inject.Inject;
import javax.inject.Singleton;

/**
 * Implementation of {@link PublicKeyRepository} and {@link PublicKeyPublisher} which stores public
 * keys in memory.
 *
 * <p>Restarting the TokenVendor will clear all keys stored with this implementation. Thus this
 * implementation should only be used in short-lived instances, e.g., during integration testing.
 */
@Singleton
class InMemoryPublicKeyRepository implements PublicKeyRepository, PublicKeyPublisher {

  // Since the current behavior of PublicKeyPublisher#publishKey is to overwrite existing keys, we
  // only have to store a single key per DeviceId rather than a list of keys.
  private final Map<DeviceId, PublicKeyPem> storedKeys = new ArrayMap<>();

  @Inject
  InMemoryPublicKeyRepository() {}

  @Override
  public ImmutableList<DeviceId> listDevices() {
    return ImmutableList.copyOf(storedKeys.keySet());
  }

  @Override
  public ImmutableList<PublicKeyPem> lookupKey(DeviceId deviceId) {
    if (storedKeys.containsKey(deviceId)) {
      return ImmutableList.of(storedKeys.get(deviceId));
    } else {
      return ImmutableList.of();
    }
  }

  @Override
  public void publishKey(DeviceId deviceId, PublicKeyPem publicKey) {
    storedKeys.put(deviceId, publicKey);
  }
}
