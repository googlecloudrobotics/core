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

/** Publishes public encryption keys of devices to a persistent storage. */
public interface PublicKeyPublisher {

  /**
   * Publishes the given public key for the device with the given ID. If a key has already been
   * published for this device it will be overwritten.
   */
  void publishKey(DeviceId deviceId, PublicKeyPem publicKey);
}
