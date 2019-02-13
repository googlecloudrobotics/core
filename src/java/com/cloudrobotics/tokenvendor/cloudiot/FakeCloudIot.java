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

import static com.google.common.base.Preconditions.checkArgument;
import static com.google.common.base.Preconditions.checkState;
import static org.hamcrest.Matchers.equalTo;
import static org.hamcrest.Matchers.hasEntry;
import static org.junit.Assert.assertThat;

import com.cloudrobotics.tokenvendor.Configuration;
import com.cloudrobotics.tokenvendor.DeviceId;
import com.cloudrobotics.tokenvendor.PublicKeyPem;
import com.google.api.client.http.HttpTransport;
import com.google.api.client.http.LowLevelHttpRequest;
import com.google.api.client.http.LowLevelHttpResponse;
import com.google.api.client.json.GenericJson;
import com.google.api.client.json.jackson2.JacksonFactory;
import com.google.api.client.testing.http.MockLowLevelHttpRequest;
import com.google.api.client.testing.http.MockLowLevelHttpResponse;
import com.google.api.services.cloudiot.v1.CloudIot;
import com.google.api.services.cloudiot.v1.model.Device;
import com.google.api.services.cloudiot.v1.model.DeviceCredential;
import com.google.api.services.cloudiot.v1.model.ListDevicesResponse;
import com.google.api.services.cloudiot.v1.model.PublicKeyCredential;
import com.google.common.base.Optional;
import com.google.common.collect.ImmutableList;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.net.HttpURLConnection;
import java.nio.charset.Charset;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.zip.GZIPInputStream;
import org.hamcrest.Description;
import org.hamcrest.TypeSafeMatcher;

/**
 * Provides an instance of {@link CloudIot} for use in tests which is backed by a fake
 * implementation of {@link HttpTransport} which stores {@link Device} instances in memory.
 *
 * <p>The fake HttpTransport updates and provides the in-memory devices by handling requests
 * according to the specification of the Cloud Iot REST API (see
 * https://cloud.google.com/iot/docs/reference/cloudiot/rest/v1/projects.locations.registries.devices).
 * However, it only supports a minimum set of endpoints and request parameters as currently needed
 * by our use of the Cloud Iot API.
 *
 * <p>Extend this class as needed!
 */
final class FakeCloudIot {

  private static final Pattern PROJECT_LOCATION_REGISTRY_PATTERN =
      Pattern.compile(
          "https://cloudiot.googleapis.com/v1"
              + "/projects/(.*)/locations/(.*)/registries/(.*)/devices(\\?(.+))?");
  private static final Pattern PROJECT_LOCATION_REGISTRY_DEVICE_PATTERN =
      Pattern.compile(
          "https://cloudiot.googleapis.com/v1"
              + "/projects/(.*)/locations/(.*)/registries/(.*)/devices/([^?]*)(\\?(.+))?");
  private static final Pattern DEVICE_ID_QUERY_PARAMETER_PATTERN =
      Pattern.compile("deviceIds=([^&=]+)");
  private static final Pattern UPDATE_MASK_QUERY_PARAMETER_PATTERN =
      Pattern.compile("updateMask=([^&=]+)");
  private static final Pattern FIELD_MASK_QUERY_PARAMETER_PATTERN =
      Pattern.compile("fieldMask=([^&=]+)");

  private static final String DEFAULT_EXPIRATION_TIME = "1970-01-01T00:00:00Z";

  private final Configuration configuration;
  private final CloudIot cloudIot;
  private final Map<DeviceId, Device> storedDevices = new HashMap<>();
  private boolean alwaysReturnErrorResponse;

  FakeCloudIot(Configuration configuration) {
    this.configuration = configuration;

    this.cloudIot =
        new CloudIot.Builder(new FakeHttpTransport(), JacksonFactory.getDefaultInstance(), null)
            .setApplicationName("test")
            .build();
  }

  CloudIot getInstance() {
    return cloudIot;
  }

  void storePublicKeysForDevice(DeviceId deviceId, PublicKeyPem... publicKeyPems) {
    String[] keysAndFormats = new String[publicKeyPems.length * 2];
    for (int i = 0; i < publicKeyPems.length; i++) {
      keysAndFormats[i * 2] = publicKeyPems[i].value();
      keysAndFormats[i * 2 + 1] = publicKeyPems[i].format().name();
    }
    storedDevices.put(deviceId, deviceWithIdAndPublicKeys(deviceId, keysAndFormats));
  }

  void storePublicKeysForDevice(DeviceId deviceId, String... keysAndFormats) {
    storedDevices.put(deviceId, deviceWithIdAndPublicKeys(deviceId, keysAndFormats));
  }

  private static Device deviceWithIdAndPublicKeys(DeviceId deviceId, String... keysAndFormats) {
    ImmutableList.Builder<DeviceCredential> deviceCredentials = ImmutableList.builder();
    for (int i = 0; i < keysAndFormats.length; i += 2) {
      deviceCredentials.add(
          new DeviceCredential()
              .setExpirationTime(DEFAULT_EXPIRATION_TIME)
              .setPublicKey(
                  new PublicKeyCredential()
                      .setKey(keysAndFormats[i])
                      .setFormat(keysAndFormats[i + 1])));
    }
    return new Device()
        .setId(deviceId.value())
        .setCredentials(deviceCredentials.build())
        .setBlocked(false);
  }

  public void setDeviceBlocked(DeviceId deviceId, boolean setBlocked) {
    storedDevices.get(deviceId).setBlocked(setBlocked);
  }

  public void setExpirationTimeForAllKeysOfDevice(DeviceId deviceId, String expirationTime) {
    storedDevices
        .get(deviceId)
        .getCredentials()
        .forEach(deviceCredential -> deviceCredential.setExpirationTime(expirationTime));
  }

  void setHttpBackendToAlwaysReturnErrorResponse(boolean enabled) {
    alwaysReturnErrorResponse = enabled;
  }

  void assertThatDeviceWithPublicKeyIsStored(DeviceId deviceId, PublicKeyPem publicKeyPem) {
    assertThat(
        storedDevices,
        hasEntry(equalTo(deviceId), aDeviceWithIdAndPublicKey(deviceId, publicKeyPem)));
  }

  private static org.hamcrest.Matcher<? super Device> aDeviceWithIdAndPublicKey(
      DeviceId deviceId, PublicKeyPem publicKeyPem) {
    return new TypeSafeMatcher<Device>() {
      @Override
      protected boolean matchesSafely(Device device) {
        return Objects.equals(device.getId(), deviceId.value())
            && device.getCredentials().size() == 1
            && Objects.equals(
                device.getCredentials().get(0).getPublicKey().getFormat(),
                publicKeyPem.format().name())
            && Objects.equals(
                device.getCredentials().get(0).getPublicKey().getKey(), publicKeyPem.value());
      }

      @Override
      public void describeTo(Description description) {
        description.appendText(
            String.format(
                "Device with id %s and public key \"%s\"", deviceId.value(), publicKeyPem.value()));
      }

      @Override
      protected void describeMismatchSafely(Device device, Description mismatchDescription) {
        try {
          mismatchDescription.appendText(device.toPrettyString());
        } catch (IOException e) {
          mismatchDescription.appendText("<failed to print device>");
        }
      }
    };
  }

  private class FakeHttpTransport extends HttpTransport {
    @Override
    protected LowLevelHttpRequest buildRequest(String method, String url) {
      if (alwaysReturnErrorResponse) {
        return new MockLowLevelHttpRequest() {
          @Override
          public LowLevelHttpResponse execute() {
            return errorBadRequestResponse("FakeCloudIot: alwaysReturnErrorResponse is enabled");
          }
        };
      } else if (isCreateDeviceRequest(method, url)) {
        return new MockLowLevelHttpRequest() {
          @Override
          public LowLevelHttpResponse execute() throws IOException {
            return handleCreateDeviceRequest(this);
          }
        };
      } else if (isListDevicesRequest(method, url)) {
        return new MockLowLevelHttpRequest() {
          @Override
          public LowLevelHttpResponse execute() throws IOException {
            return handleListDevicesRequest(url);
          }
        };
      } else if (isGetDeviceRequest(method, url)) {
        return new MockLowLevelHttpRequest() {
          @Override
          public LowLevelHttpResponse execute() throws IOException {
            return handleGetDeviceRequest(url);
          }
        };
      } else if (isPatchDeviceRequest(method, url)) {
        return new MockLowLevelHttpRequest() {
          @Override
          public LowLevelHttpResponse execute() throws IOException {
            return handlePatchDeviceRequest(url, this);
          }
        };
      } else {
        throw new IllegalArgumentException(
            String.format("FakeCloudIot: Unexpected %s call to %s", method, url));
      }
    }
  }

  boolean isCreateDeviceRequest(String method, String url) {
    return method.equals("POST") && urlMatchesPattern(url, PROJECT_LOCATION_REGISTRY_PATTERN);
  }

  boolean isListDevicesRequest(String method, String url) {
    return method.equals("GET") && urlMatchesPattern(url, PROJECT_LOCATION_REGISTRY_PATTERN);
  }

  boolean isGetDeviceRequest(String method, String url) {
    return method.equals("GET") && urlMatchesPattern(url, PROJECT_LOCATION_REGISTRY_DEVICE_PATTERN);
  }

  boolean isPatchDeviceRequest(String method, String url) {
    // What should be a PATCH call is actually executed as a POST call by the CloudIot class (see
    // https://cloud.google.com/iot/docs/reference/cloudiot/rest/v1/projects.locations.registries.devices/patch).
    return method.equals("POST")
        && urlMatchesPattern(url, PROJECT_LOCATION_REGISTRY_DEVICE_PATTERN);
  }

  private boolean urlMatchesPattern(String url, Pattern pattern) {
    Matcher matcher = pattern.matcher(url);
    if (matcher.matches()) {
      checkProjectNameAndRegionAndRegistry(matcher.group(1), matcher.group(2), matcher.group(3));
      return true;
    } else {
      return false;
    }
  }

  private void checkProjectNameAndRegionAndRegistry(
      String projectName, String region, String registry) {
    checkArgument(
        Objects.equals(configuration.projectName, projectName),
        "Expected request for project ID %s, but got %s",
        configuration.projectName,
        projectName);
    checkArgument(
        Objects.equals(configuration.cloudRegion, region),
        "Expected request for region %s, but got %s",
        configuration.cloudRegion,
        region);
    checkArgument(
        Objects.equals(configuration.registryName, registry),
        "Expected request for registry name %s, but got %s",
        configuration.registryName,
        registry);
  }

  private LowLevelHttpResponse handleCreateDeviceRequest(MockLowLevelHttpRequest request)
      throws IOException {
    Device deviceToCreate = parseDeviceFromRequestBody(request);

    if (deviceToCreate.getNumId() != null) {
      return errorBadRequestResponse("Device to be created must not have numId set");
    } else if (deviceToCreate.getId() == null) {
      return errorBadRequestResponse("Device to be created must have id set");
    } else if (storedDevices.containsKey(DeviceId.of(deviceToCreate.getId()))) {
      return errorBadRequestResponse(
          String.format("A device with the id %s already exists", deviceToCreate.getId()));
    } else {
      storedDevices.put(DeviceId.of(deviceToCreate.getId()), deviceToCreate);
      return okResponseWithJsonContent(deviceToCreate);
    }
  }

  private LowLevelHttpResponse handlePatchDeviceRequest(String url, MockLowLevelHttpRequest request)
      throws IOException {
    DeviceId deviceId = getDeviceIdFromPath(url);
    checkUpdateMaskQueryParameter(url);

    if (storedDevices.containsKey(deviceId)) {
      Device patchedDevice = parseDeviceFromRequestBody(request);

      if (patchedDevice.getId() != null) {
        return errorBadRequestResponse(
            String.format(
                "The 'device.id' and 'device.num_id' fields must be empty. Got device.id='%s'",
                patchedDevice.getId()));
      }
      if (patchedDevice.getNumId() != null) {
        return errorBadRequestResponse(
            String.format(
                "The 'device.id' and 'device.num_id' fields must be empty. Got device.num_id='%s'",
                patchedDevice.getNumId()));
      }

      patchedDevice.setId(deviceId.value());
      storedDevices.put(deviceId, patchedDevice);

      return okResponseWithJsonContent(patchedDevice);
    } else {
      return errorNotFoundResponse();
    }
  }

  private static void checkUpdateMaskQueryParameter(String url) {
    Matcher urlMatcher = PROJECT_LOCATION_REGISTRY_DEVICE_PATTERN.matcher(url);
    checkState(urlMatcher.matches());

    String queryParameters = urlMatcher.group(6);
    Matcher queryParametersMatcher = UPDATE_MASK_QUERY_PARAMETER_PATTERN.matcher(queryParameters);

    checkArgument(
        queryParametersMatcher.matches() && queryParametersMatcher.group(1).equals("credentials"),
        "The 'update_mask' must contain mutable fields. Fields ['blocked', 'credentials', "
            + "'gateway_config.gateway_auth_method', 'log_level', 'metadata'] are mutable. "
            + "This fake implementation currently only supports 'credentials' (but received '%s').",
        queryParameters);
  }

  private LowLevelHttpResponse handleGetDeviceRequest(String url) throws IOException {
    DeviceId deviceId = getDeviceIdFromPath(url);
    checkFieldMaskContainsCredentialsAndBlocked(url);

    if (storedDevices.containsKey(deviceId)) {
      return okResponseWithJsonContent(storedDevices.get(deviceId));
    } else {
      return errorNotFoundResponse();
    }
  }

  private static void checkFieldMaskContainsCredentialsAndBlocked(String url) {
    Matcher urlMatcher = PROJECT_LOCATION_REGISTRY_DEVICE_PATTERN.matcher(url);
    checkState(urlMatcher.matches());

    String queryParameters = urlMatcher.group(6);
    Matcher queryParametersMatcher = FIELD_MASK_QUERY_PARAMETER_PATTERN.matcher(queryParameters);

    checkArgument(
        queryParametersMatcher.matches()
            && queryParametersMatcher.group(1).equals("credentials,blocked"),
        "This fake implementation currently only supports "
            + "'fieldMask=credentials,blocked' (but received '%s').",
        queryParameters);
  }

  private static DeviceId getDeviceIdFromPath(String url) {
    Matcher urlMatcher = PROJECT_LOCATION_REGISTRY_DEVICE_PATTERN.matcher(url);
    checkState(urlMatcher.matches());

    return DeviceId.of(urlMatcher.group(4));
  }

  private LowLevelHttpResponse handleListDevicesRequest(String url) throws IOException {
    Optional<DeviceId> requestedDeviceId = getDeviceIdFromQueryParameters(url);
    if (requestedDeviceId.isPresent()) {
      if (storedDevices.containsKey(requestedDeviceId.get())) {
        return okResponseWithJsonContent(
            new ListDevicesResponse()
                .setDevices(ImmutableList.of(storedDevices.get(requestedDeviceId.get()))));
      } else {
        return okResponseWithJsonContent(new ListDevicesResponse());
      }
    } else {
      return okResponseWithJsonContent(
          new ListDevicesResponse().setDevices(ImmutableList.copyOf(storedDevices.values())));
    }
  }

  private static Optional<DeviceId> getDeviceIdFromQueryParameters(String url) {
    Matcher urlMatcher = PROJECT_LOCATION_REGISTRY_PATTERN.matcher(url);
    checkState(urlMatcher.matches());

    String queryParameters = urlMatcher.group(5);
    if (queryParameters == null || queryParameters.equals("")) {
      return Optional.absent();
    }
    Matcher queryParametersMatcher = DEVICE_ID_QUERY_PARAMETER_PATTERN.matcher(queryParameters);

    checkArgument(
        queryParametersMatcher.matches(),
        "Currently only query parameters matching \"%s\" are supported, received %s",
        DEVICE_ID_QUERY_PARAMETER_PATTERN,
        queryParameters);

    return Optional.of(DeviceId.of(queryParametersMatcher.group(1)));
  }

  private static Device parseDeviceFromRequestBody(MockLowLevelHttpRequest request)
      throws IOException {
    checkArgument(
        Objects.equals(request.getContentType(), "application/json; charset=UTF-8"),
        "Support for content other than UTF-8 encoded JSON not implemented yet.");
    checkArgument(
        Objects.equals(request.getHeaders().get("accept-encoding"), ImmutableList.of("gzip")),
        "Support for unzipped request bodies not implemented yet.");

    ByteArrayOutputStream gzippedOutputStream = new ByteArrayOutputStream();
    request.getStreamingContent().writeTo(gzippedOutputStream);

    ByteArrayInputStream gzippedInputStream =
        new ByteArrayInputStream(gzippedOutputStream.toByteArray());
    GZIPInputStream unzippedInputStream = new GZIPInputStream(gzippedInputStream);

    return JacksonFactory.getDefaultInstance()
        .fromInputStream(unzippedInputStream, Charset.forName("UTF-8"), Device.class);
  }

  private static LowLevelHttpResponse okResponseWithJsonContent(GenericJson jsonContent)
      throws IOException {
    return responseWithErrorCodeAndJsonContent(
        HttpURLConnection.HTTP_OK, JacksonFactory.getDefaultInstance().toString(jsonContent));
  }

  private static LowLevelHttpResponse errorBadRequestResponse(String message) {
    return responseWithErrorCodeAndJsonContent(
        HttpURLConnection.HTTP_BAD_REQUEST,
        String.format(
            "{ \"error\": { \"code\": %d, \"message\": \"%s\" } }",
            HttpURLConnection.HTTP_BAD_REQUEST, message));
  }

  private static LowLevelHttpResponse errorNotFoundResponse() {
    return responseWithErrorCodeAndJsonContent(
        HttpURLConnection.HTTP_NOT_FOUND,
        String.format(
            "{ \"error\": { \"code\": %d, \"message\": \"not found\" } }",
            HttpURLConnection.HTTP_NOT_FOUND));
  }

  private static LowLevelHttpResponse responseWithErrorCodeAndJsonContent(
      int statusCode, String jsonContent) {
    MockLowLevelHttpResponse response = new MockLowLevelHttpResponse();
    response.setStatusCode(statusCode);
    response.setContentType("application/json");
    response.setContent(jsonContent);
    return response;
  }
}
