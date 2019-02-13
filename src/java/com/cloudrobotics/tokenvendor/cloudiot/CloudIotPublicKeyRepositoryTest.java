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

package com.cloudrobotics.tokenvendor.cloudiot;

import static org.hamcrest.CoreMatchers.is;
import static org.hamcrest.Matchers.contains;
import static org.hamcrest.Matchers.containsString;
import static org.hamcrest.Matchers.empty;
import static org.hamcrest.Matchers.hasSize;
import static org.junit.Assert.assertThat;
import static org.junit.jupiter.api.Assertions.assertThrows;

import com.cloudrobotics.tokenvendor.Configuration;
import com.cloudrobotics.tokenvendor.DeviceId;
import com.cloudrobotics.tokenvendor.PublicKeyPem;
import com.google.common.collect.ImmutableList;
import java.time.Instant;
import java.time.temporal.ChronoUnit;
import junitparams.JUnitParamsRunner;
import junitparams.Parameters;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;

/** Tests of {@link CloudIotPublicKeyRepository} */
@RunWith(JUnitParamsRunner.class)
public class CloudIotPublicKeyRepositoryTest {

  private static final DeviceId DEVICE_ID = DeviceId.of("device-id");
  private static final String DEFAULT_TIMESTAMP = "2018-10-20T10:00:00.00Z";

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

  private FakeCloudIot cloudIot;

  private CloudIotPublicKeyRepository repository;

  private Instant currentTime = Instant.parse(DEFAULT_TIMESTAMP);

  @Before
  public void setUp() {
    Configuration configuration = new Configuration();
    configuration.projectName = "cloud-project";
    configuration.cloudRegion = "europe-west1";
    configuration.registryName = "my-registry";

    cloudIot = new FakeCloudIot(configuration);

    repository =
        new CloudIotPublicKeyRepository(cloudIot.getInstance(), configuration, () -> currentTime);
  }

  @Test
  public void publishKeyShouldPublishKeyForNewDevice() {
    repository.publishKey(DEVICE_ID, PUBLIC_KEY_PEM);

    cloudIot.assertThatDeviceWithPublicKeyIsStored(DEVICE_ID, PUBLIC_KEY_PEM);
  }

  @Test
  public void publishKeyShouldUpdateKeyForExistingDevice() {
    cloudIot.storePublicKeysForDevice(DEVICE_ID, PUBLIC_KEY_PEM);

    repository.publishKey(DEVICE_ID, ANOTHER_PUBLIC_KEY_PEM);

    cloudIot.assertThatDeviceWithPublicKeyIsStored(DEVICE_ID, ANOTHER_PUBLIC_KEY_PEM);
  }

  @Test
  public void publishKeyShouldThrowIfRequestFails() {
    cloudIot.setHttpBackendToAlwaysReturnErrorResponse(true);

    assertThrows(CloudIotException.class, () -> repository.publishKey(DEVICE_ID, PUBLIC_KEY_PEM));
  }

  @Test
  public void lookupKeyShouldReturnStoredPublicKeys() {
    cloudIot.storePublicKeysForDevice(DEVICE_ID, PUBLIC_KEY_PEM, ANOTHER_PUBLIC_KEY_PEM);

    ImmutableList<PublicKeyPem> result = repository.lookupKey(DEVICE_ID);

    assertThat(result, contains(PUBLIC_KEY_PEM, ANOTHER_PUBLIC_KEY_PEM));
  }

  @Test
  public void lookupKeyShouldReturnEmptyListIfDeviceDoesNotExist() {
    ImmutableList<PublicKeyPem> result = repository.lookupKey(DEVICE_ID);

    assertThat(result, is(empty()));
  }

  @Test
  public void lookupKeyShouldThrowIfRequestFails() {
    cloudIot.setHttpBackendToAlwaysReturnErrorResponse(true);

    assertThrows(CloudIotException.class, () -> repository.lookupKey(DEVICE_ID));
  }

  @Test
  public void lookupKeyShouldReturnEmptyListIfDeviceBlocked() {
    cloudIot.storePublicKeysForDevice(DEVICE_ID, PUBLIC_KEY_PEM);
    cloudIot.setDeviceBlocked(DEVICE_ID, true);

    ImmutableList<PublicKeyPem> result = repository.lookupKey(DEVICE_ID);

    assertThat(result, is(empty()));
  }

  @Test
  public void lookupKeyShouldReturnKeyIfKeyHasNotExpired() {
    currentTime = Instant.parse(DEFAULT_TIMESTAMP);
    Instant expirationTime = currentTime.plus(1, ChronoUnit.SECONDS);
    cloudIot.storePublicKeysForDevice(DEVICE_ID, PUBLIC_KEY_PEM);
    cloudIot.setExpirationTimeForAllKeysOfDevice(DEVICE_ID, expirationTime.toString());

    ImmutableList<PublicKeyPem> result = repository.lookupKey(DEVICE_ID);

    assertThat(result, contains(PUBLIC_KEY_PEM));
  }

  @Test
  public void lookupKeyShouldReturnEmptyListIfKeyHasExpired() {
    currentTime = Instant.parse(DEFAULT_TIMESTAMP);
    Instant expirationTime = currentTime.minus(1, ChronoUnit.SECONDS);
    cloudIot.storePublicKeysForDevice(DEVICE_ID, PUBLIC_KEY_PEM);
    cloudIot.setExpirationTimeForAllKeysOfDevice(DEVICE_ID, expirationTime.toString());

    ImmutableList<PublicKeyPem> result = repository.lookupKey(DEVICE_ID);

    assertThat(result, is(empty()));
  }

  @Test
  @Parameters(method = "formatCases")
  public void lookupKeyShouldReturnCorrectKeyFormat(PublicKeyPem.Format format) {
    cloudIot.storePublicKeysForDevice(DEVICE_ID, PUBLIC_KEY_PEM.value(), format.name());

    ImmutableList<PublicKeyPem> result = repository.lookupKey(DEVICE_ID);

    assertThat(result, hasSize(1));
    assertThat(result.get(0).format(), is(format));
  }

  @Parameters
  public static Object[] formatCases() {
    return PublicKeyPem.Format.values();
  }

  @Test
  public void lookupKeyShouldThrowOnUnsupportedKeyFormat() {
    cloudIot.storePublicKeysForDevice(DEVICE_ID, PUBLIC_KEY_PEM.value(), "UNSUPPORTED_FORMAT");

    CloudIotException exception =
        assertThrows(CloudIotException.class, () -> repository.lookupKey(DEVICE_ID));

    assertThat(
        exception.getMessage(),
        containsString("Encountered public key in an unsupported format: UNSUPPORTED_FORMAT"));
  }

  @Test
  public void listDevicesShouldReturnEmptyListForEmptyRegistry() {
    ImmutableList<DeviceId> result = repository.listDevices();

    assertThat(result, is(empty()));
  }

  @Test
  public void listDevicesShouldListRegisteredDevices() {
    cloudIot.storePublicKeysForDevice(DEVICE_ID, PUBLIC_KEY_PEM);

    ImmutableList<DeviceId> result = repository.listDevices();

    assertThat(result, hasSize(1));
    assertThat(result.get(0), is(DEVICE_ID));
  }
}
