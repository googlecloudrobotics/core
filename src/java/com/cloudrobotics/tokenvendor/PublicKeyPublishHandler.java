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

import static com.google.common.base.Preconditions.checkArgument;

import com.google.common.base.Charsets;
import com.google.common.base.Splitter;
import com.google.common.io.CharStreams;
import com.google.common.net.HttpHeaders;
import com.sun.net.httpserver.Headers;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.util.Map;
import javax.inject.Inject;
import org.apache.commons.lang.exception.ExceptionUtils;

/**
 * Handles HTTP requests to publish a device public key by using a {@link PublicKeyPublisher}.
 *
 * <p>Currently this endpoint only supports public RSA keys.
 */
final class PublicKeyPublishHandler implements HttpHandler {

  private static final String DEVICE_ID = "device-id";

  private final PublicKeyPublisher publisher;

  @Inject
  PublicKeyPublishHandler(PublicKeyPublisher publisher) {
    this.publisher = publisher;
  }

  @Override
  public void handle(HttpExchange exchange) throws IOException {
    try {
      handleOrThrow(exchange);
      setOkResponse(exchange);
    } catch (IllegalArgumentException e) {
      setResponseWithCodeAndMessage(HttpURLConnection.HTTP_BAD_REQUEST, e.getMessage(), exchange);
    } catch (Exception e) {
      setResponseWithCodeAndMessage(
          HttpURLConnection.HTTP_INTERNAL_ERROR,
          "Internal server error:\n" + ExceptionUtils.getStackTrace(e),
          exchange);
    }
  }

  private void handleOrThrow(HttpExchange exchange) throws IOException {
    checkArgument(
        exchange.getRequestMethod().equals("POST"), "This path only supports POST requests");
    DeviceId deviceId = getDeviceId(exchange);
    PublicKeyPem publicKey =
        PublicKeyPem.of(getPemRequestBody(exchange), PublicKeyPem.Format.RSA_PEM);
    publisher.publishKey(deviceId, publicKey);
  }

  static Map<String, String> splitQueryString(String query) {
    try {
      return Splitter.on('&').withKeyValueSeparator('=').split(query);
    } catch (IllegalArgumentException e) {
      throw new IllegalArgumentException("Missing query parameter: " + DEVICE_ID, e);
    }
  }

  private static DeviceId getDeviceId(HttpExchange exchange) {
    String query = exchange.getRequestURI().getQuery();
    checkArgument(query != null, "Missing query parameter: " + DEVICE_ID);
    Map<String, String> queryParameters = splitQueryString(query);
    checkArgument(queryParameters.get(DEVICE_ID) != null, "Missing query parameter: " + DEVICE_ID);
    return DeviceId.of(queryParameters.get(DEVICE_ID));
  }

  private static String getPemRequestBody(HttpExchange exchange) throws IOException {
    Headers requestHeaders = exchange.getRequestHeaders();
    checkArgument(
        requestHeaders.containsKey(HttpHeaders.CONTENT_TYPE)
            && requestHeaders.get(HttpHeaders.CONTENT_TYPE).get(0).equals("application/x-pem-file"),
        "Expected content type \"application/x-pem-file\"");

    return CharStreams.toString(new InputStreamReader(exchange.getRequestBody(), Charsets.UTF_8));
  }

  private static void setOkResponse(HttpExchange exchange) throws IOException {
    setResponseWithCodeAndMessage(HttpURLConnection.HTTP_OK, "OK", exchange);
  }

  private static void setResponseWithCodeAndMessage(
      int errorCode, String message, HttpExchange exchange) throws IOException {
    setResponseWithCodeMessageAndContentType(
        errorCode, message, "text/html; charset=UTF-8", exchange);
  }

  static void setResponseWithCodeMessageAndContentType(
      int errorCode, String message, String contentType, HttpExchange exchange) throws IOException {
    exchange.getResponseHeaders().add(HttpHeaders.CONTENT_TYPE, contentType);
    exchange.sendResponseHeaders(errorCode, 0);
    exchange.getResponseBody().write(message.getBytes(Charsets.UTF_8));
    exchange.getResponseBody().close();
  }
}
