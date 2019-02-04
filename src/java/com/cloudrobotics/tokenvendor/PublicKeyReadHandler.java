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

import static com.google.common.base.Preconditions.checkArgument;

import com.google.common.base.Optional;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import java.io.IOException;
import java.net.HttpURLConnection;
import java.util.Map;
import java.util.stream.Collectors;
import javax.inject.Inject;
import org.apache.commons.lang.exception.ExceptionUtils;

/** Handles HTTP requests to read public keys of a device by using a {@link PublicKeyRepository}. */
final class PublicKeyReadHandler implements HttpHandler {

  private static final String DEVICE_ID = "device-id";

  private final PublicKeyRepository repository;

  @Inject
  PublicKeyReadHandler(PublicKeyRepository repository) {
    this.repository = repository;
  }

  @Override
  public void handle(HttpExchange exchange) throws IOException {
    try {
      String keys = handleOrThrow(exchange);
      setOkResponse(exchange, keys);
    } catch (IllegalArgumentException e) {
      setResponseWithCodeAndMessage(HttpURLConnection.HTTP_BAD_REQUEST, e.getMessage(), exchange);
    } catch (Exception e) {
      setResponseWithCodeAndMessage(
          HttpURLConnection.HTTP_INTERNAL_ERROR,
          "Internal server error:\n" + ExceptionUtils.getStackTrace(e),
          exchange);
    }
  }

  /* TODO(ensonic): if no device-id param is used we return all public keys. Design some filtering
   * to select e.g. only dev-keys or only robot-keys */
  private String handleOrThrow(HttpExchange exchange) {
    checkArgument(
        exchange.getRequestMethod().equals("GET"), "This path only supports GET requests");
    Optional<DeviceId> deviceId = getDeviceId(exchange);
    if (deviceId.isPresent()) {
      return getKeysForDevice(deviceId.get());
    } else {
      return getKeysForAllDevices();
    }
  }

  private String getKeysForDevice(DeviceId deviceId) {
    return repository.lookupKey(deviceId).stream()
        .map(PublicKeyPem::value)
        .collect(Collectors.joining("\n"));
  }

  private String getKeysForAllDevices() {
    return repository.listDevices().stream()
        .map(d -> getKeysForDevice(d))
        .collect(Collectors.joining());
  }

  private static Optional<DeviceId> getDeviceId(HttpExchange exchange) {
    String query = exchange.getRequestURI().getQuery();
    if (query == null) {
      return Optional.absent();
    }
    try {
      Map<String, String> queryParameters = PublicKeyPublishHandler.splitQueryString(query);
      if (!queryParameters.containsKey(DEVICE_ID) || queryParameters.get(DEVICE_ID).equals("")) {
        return Optional.absent();
      }
      return Optional.of(DeviceId.of(queryParameters.get(DEVICE_ID)));
    } catch (IllegalArgumentException e) {
      return Optional.absent();
    }
  }

  private static void setOkResponse(HttpExchange exchange, String message) throws IOException {
    PublicKeyPublishHandler.setResponseWithCodeMessageAndContentType(
        HttpURLConnection.HTTP_OK, message, "application/x-pem-file", exchange);
  }

  private static void setResponseWithCodeAndMessage(
      int errorCode, String message, HttpExchange exchange) throws IOException {
    PublicKeyPublishHandler.setResponseWithCodeMessageAndContentType(
        errorCode, message, "text/html; charset=UTF-8", exchange);
  }
}
