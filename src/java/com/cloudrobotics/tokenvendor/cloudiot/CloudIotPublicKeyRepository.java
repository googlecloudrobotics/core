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

import static com.google.common.collect.ImmutableList.toImmutableList;

import com.cloudrobotics.framework.Clock;
import com.cloudrobotics.tokenvendor.Configuration;
import com.cloudrobotics.tokenvendor.DeviceId;
import com.cloudrobotics.tokenvendor.PublicKeyPem;
import com.cloudrobotics.tokenvendor.PublicKeyPublisher;
import com.cloudrobotics.tokenvendor.PublicKeyRepository;
import com.google.api.client.googleapis.json.GoogleJsonResponseException;
import com.google.api.services.cloudiot.v1.CloudIot;
import com.google.api.services.cloudiot.v1.model.Device;
import com.google.api.services.cloudiot.v1.model.DeviceCredential;
import com.google.api.services.cloudiot.v1.model.ListDevicesResponse;
import com.google.api.services.cloudiot.v1.model.PublicKeyCredential;
import com.google.common.collect.ImmutableList;
import com.google.common.flogger.FluentLogger;
import java.io.IOException;
import java.net.HttpURLConnection;
import java.time.Instant;
import java.util.List;
import javax.inject.Inject;
import javax.inject.Singleton;

/**
 * Implementation of {@link PublicKeyRepository} and {@link PublicKeyPublisher} which stores public
 * keys to the Google Cloud Iot Registry via the corresponding Cloud API.
 */
@Singleton
class CloudIotPublicKeyRepository implements PublicKeyPublisher, PublicKeyRepository {

  private static final FluentLogger logger = FluentLogger.forEnclosingClass();

  private final CloudIot cloudIot;
  private final Configuration configuration;
  private final Clock clock;

  @Inject
  CloudIotPublicKeyRepository(CloudIot cloudIot, Configuration configuration, Clock clock) {
    this.cloudIot = cloudIot;
    this.configuration = configuration;
    this.clock = clock;
  }

  @Override
  public void publishKey(DeviceId deviceId, PublicKeyPem publicKeyPem) {
    // TODO(b/73715747): These devices are never cleaned up.
    try {
      if (deviceAlreadyExists(deviceId)) {
        updateDevice(deviceId, publicKeyPem);
      } else {
        createDevice(deviceId, publicKeyPem);
      }
    } catch (IOException e) {
      String message =
          String.format(
              "Failed to publish public key for device %s to Cloud IoT Registry", deviceId.value());
      logger.atSevere().withCause(e).log("%s", message);
      throw new CloudIotException(message, e);
    }
  }

  private boolean deviceAlreadyExists(DeviceId deviceId) throws IOException {
    ListDevicesResponse existingDevices =
        cloudIot
            .projects()
            .locations()
            .registries()
            .devices()
            .list(pathWithoutDeviceId())
            .setDeviceIds(ImmutableList.of(deviceId.value()))
            .execute();

    return existingDevices.getDevices() != null && existingDevices.getDevices().size() > 0;
  }

  private void updateDevice(DeviceId deviceId, PublicKeyPem publicKeyPem) throws IOException {
    Device device = deviceWithPublicKey(publicKeyPem);
    cloudIot
        .projects()
        .locations()
        .registries()
        .devices()
        .patch(pathWithDeviceId(deviceId), device)
        .setUpdateMask("credentials")
        .execute();

    logger.atInfo().log(
        "Successfully updated public key for device %s to Cloud IoT Registry (new key: %s)",
        deviceId.value(), publicKeyPem.value());
  }

  private void createDevice(DeviceId deviceId, PublicKeyPem publicKeyPem) throws IOException {
    Device device = deviceWithIdAndPublicKey(deviceId, publicKeyPem);
    cloudIot
        .projects()
        .locations()
        .registries()
        .devices()
        .create(pathWithoutDeviceId(), device)
        .execute();

    logger.atInfo().log(
        "Successfully published public key for device %s to Cloud IoT Registry (new key: %s)",
        deviceId.value(), publicKeyPem.value());
  }

  private static Device deviceWithIdAndPublicKey(DeviceId deviceId, PublicKeyPem publicKeyPem) {
    Device device = deviceWithPublicKey(publicKeyPem);
    device.setId(deviceId.value());
    return device;
  }

  private static Device deviceWithPublicKey(PublicKeyPem publicKeyPem) {
    return new Device()
        .setCredentials(
            ImmutableList.of(
                new DeviceCredential()
                    .setPublicKey(
                        new PublicKeyCredential()
                            .setFormat(publicKeyPem.format().name())
                            .setKey(publicKeyPem.value()))));
  }

  @Override
  public ImmutableList<DeviceId> listDevices() {
    try {
      List<Device> devices =
          cloudIot
              .projects()
              .locations()
              .registries()
              .devices()
              .list(pathWithoutDeviceId())
              .execute()
              .getDevices();

      return devices.stream()
          .map(d -> DeviceId.of(d.getId()))
          .collect(ImmutableList.toImmutableList());

    } catch (IOException e) {
      if (e instanceof GoogleJsonResponseException
          && ((GoogleJsonResponseException) e).getDetails().getCode()
              == HttpURLConnection.HTTP_NOT_FOUND) {
        return ImmutableList.of();
      }
      String message = String.format("Failed to list devices from Cloud IoT Registry");
      logger.atSevere().withCause(e).log("%s", message);
      throw new CloudIotException(message, e);
    }
  }

  @Override
  public ImmutableList<PublicKeyPem> lookupKey(DeviceId deviceId) {
    try {
      Device device =
          cloudIot
              .projects()
              .locations()
              .registries()
              .devices()
              .get(pathWithDeviceId(deviceId))
              .setFieldMask("credentials,blocked")
              .execute();
      return getValidKeys(device);
    } catch (IOException e) {
      if (e instanceof GoogleJsonResponseException
          && ((GoogleJsonResponseException) e).getDetails().getCode()
              == HttpURLConnection.HTTP_NOT_FOUND) {
        return ImmutableList.of();
      }
      String message =
          String.format("Failed to get device %s from Cloud IoT Registry", deviceId.value());
      logger.atSevere().withCause(e).log("%s", message);
      throw new CloudIotException(message, e);
    }
  }

  private ImmutableList<PublicKeyPem> getValidKeys(Device device) {
    // Device blockage is an administrative state that's designed to prevent
    // devices that might be stolen from authorizing.
    if (device.getBlocked() != null && device.getBlocked()) {
      return ImmutableList.of();
    }

    return device.getCredentials().stream()
        .filter(deviceCredential -> !hasExpired(deviceCredential))
        .map(deviceCredential -> toPublicKeyPem(deviceCredential))
        .collect(toImmutableList());
  }

  private boolean hasExpired(DeviceCredential deviceCredential) {
    Instant expirationTime = Instant.parse(deviceCredential.getExpirationTime());
    // Cloud IoT uses a default expiration time of 0  for keys that won't expire
    return expirationTime.getEpochSecond() != 0 && clock.now().isAfter(expirationTime);
  }

  private static PublicKeyPem toPublicKeyPem(DeviceCredential deviceCredential) {
    PublicKeyPem.Format format;
    try {
      format = PublicKeyPem.Format.valueOf(deviceCredential.getPublicKey().getFormat());
    } catch (IllegalArgumentException e) {
      throw new CloudIotException(
          String.format(
              "Encountered public key in an unsupported format: %s",
              deviceCredential.getPublicKey().getFormat()),
          e);
    }
    return PublicKeyPem.of(deviceCredential.getPublicKey().getKey(), format);
  }

  private String pathWithoutDeviceId() {
    return String.format(
        "projects/%s/locations/%s/registries/%s",
        configuration.projectName, configuration.cloudRegion, configuration.registryName);
  }

  private String pathWithDeviceId(DeviceId deviceId) {
    return String.format(
        "projects/%s/locations/%s/registries/%s/devices/%s",
        configuration.projectName,
        configuration.cloudRegion,
        configuration.registryName,
        deviceId.value());
  }
}
