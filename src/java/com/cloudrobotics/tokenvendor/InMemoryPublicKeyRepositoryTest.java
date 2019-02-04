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

import static org.hamcrest.CoreMatchers.is;
import static org.hamcrest.Matchers.contains;
import static org.hamcrest.Matchers.empty;
import static org.hamcrest.Matchers.hasSize;
import static org.junit.Assert.assertThat;

import com.google.common.collect.ImmutableList;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

/** Tests of {@link InMemoryPublicKeyRepository}. */
@RunWith(JUnit4.class)
public class InMemoryPublicKeyRepositoryTest {

  private static final DeviceId DEVICE_ID = DeviceId.of("my-device");

  private static final PublicKeyPem PUBLIC_KEY_PEM =
      PublicKeyPem.of(
          "-----BEGIN PUBLIC KEY-----\n"
              + "MFwwDQYJKoZIhvcNAQEBBQADSwAwSAJBAID64aLO4Gui+Z3uRL0iu/zO6H+b6/jMGPcf"
              + "Gav4Xk5SS8wNEciSwT80Bbu5p2cBFcTHmnsaVhbgfkaeWefEiGcCAwEAAQ=="
              + "\n-----END PUBLIC KEY-----",
          PublicKeyPem.Format.RSA_PEM);

  private static final PublicKeyPem ANOTHER_PUBLIC_KEY_PEM =
      PublicKeyPem.of(
          "-----BEGIN PUBLIC KEY-----\n"
              + "MFwwDQYJKoZIhvcNAQEBBQADSwAwSAJBAI3jwOZ9YTNV1x3NRLmXOMcgd15zZPbc"
              + "/L69Hh1tDd7iBgSS8XBzloynWaqRDsqXizf9jaCX2kFHPbSEDyntQo8CAwEAAQ=="
              + "\n-----END PUBLIC KEY-----",
          PublicKeyPem.Format.ES256_PEM);

  InMemoryPublicKeyRepository publicKeyRepository = new InMemoryPublicKeyRepository();

  @Test
  public void lookupKeyReturnsEmptyListIfNoKeyStored() {
    ImmutableList<PublicKeyPem> keys = publicKeyRepository.lookupKey(DEVICE_ID);

    assertThat(keys, is(empty()));
  }

  @Test
  public void lookupKeyReturnsStoredKey() {
    publicKeyRepository.publishKey(DEVICE_ID, PUBLIC_KEY_PEM);

    ImmutableList<PublicKeyPem> keys = publicKeyRepository.lookupKey(DEVICE_ID);

    assertThat(keys, contains(PUBLIC_KEY_PEM));
  }

  @Test
  public void publishKeyStoresKey() {
    publicKeyRepository.publishKey(DEVICE_ID, PUBLIC_KEY_PEM);

    assertThat(publicKeyRepository.lookupKey(DEVICE_ID), contains(PUBLIC_KEY_PEM));
  }

  @Test
  public void publishKeyOverridesStoredKey() {
    publicKeyRepository.publishKey(DEVICE_ID, PUBLIC_KEY_PEM);

    publicKeyRepository.publishKey(DEVICE_ID, ANOTHER_PUBLIC_KEY_PEM);

    assertThat(publicKeyRepository.lookupKey(DEVICE_ID), contains(ANOTHER_PUBLIC_KEY_PEM));
  }

  @Test
  public void listDevicesShouldReturnEmptyListForEmptyRegistry() {
    ImmutableList<DeviceId> result = publicKeyRepository.listDevices();

    assertThat(result, is(empty()));
  }

  @Test
  public void listDevicesShouldListRegisteredDevices() {
    publicKeyRepository.publishKey(DEVICE_ID, PUBLIC_KEY_PEM);

    ImmutableList<DeviceId> result = publicKeyRepository.listDevices();

    assertThat(result, hasSize(1));
    assertThat(result.get(0), is(DEVICE_ID));
  }
}
