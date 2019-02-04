// Copyright 2019 The Google Cloud Robotics Authors
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

import com.google.common.collect.ImmutableList;

/** Provides read access to a persistent storage of public encryption keys from devices. */
public interface PublicKeyRepository {

  /** Lists all device ids currently in the registry. */
  ImmutableList<DeviceId> listDevices();

  /**
   * Looks up all <i>valid<i/> public keys which are currently stored for the given {@link DeviceId}
   * and returns them as a list. Returns an empty list if no entry for the given DeviceId exists or
   * if no valid key is stored for the given DeviceId.
   *
   * <p>Depending on the concrete storage system used to store the public keys, there might be,
   * e.g., support for blocking keys by an administrator or keys might be stored with expiry dates.
   * Implementations of this method should filter out such invalid keys from the result.
   */
  ImmutableList<PublicKeyPem> lookupKey(DeviceId deviceId);
}
