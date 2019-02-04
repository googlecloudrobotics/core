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

import static org.hamcrest.CoreMatchers.containsString;
import static org.hamcrest.CoreMatchers.is;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThat;
import static org.mockito.Mockito.when;

import com.cloudrobotics.framework.HttpServerTestRule;
import com.google.common.base.Charsets;
import com.google.common.collect.ImmutableList;
import com.google.common.io.CharStreams;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;
import junitparams.JUnitParamsRunner;
import junitparams.Parameters;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.Mock;
import org.mockito.junit.MockitoJUnit;
import org.mockito.junit.MockitoRule;

/** Tests of {@link PublicKeyReadHandler}. */
@RunWith(JUnitParamsRunner.class)
public class PublicKeyReadHandlerTest {

  private static final DeviceId DEVICE_ID1 = DeviceId.of("my-robot1");
  private static final DeviceId DEVICE_ID2 = DeviceId.of("my-robot2");

  private static final PublicKeyPem PUBLIC_KEY_PEM =
      PublicKeyPem.of(
          "-----BEGIN PUBLIC KEY-----\n"
              + "MFwwDQYJKoZIhvcNAQEBBQADSwAwSAJBAID64aLO4Gui+Z3uRL0iu/zO6H+b6/jMGPcf"
              + "Gav4Xk5SS8wNEciSwT80Bbu5p2cBFcTHmnsaVhbgfkaeWefEiGcCAwEAAQ=="
              + "\n-----END PUBLIC KEY-----",
          PublicKeyPem.Format.RSA_PEM);

  @Rule public MockitoRule rule = MockitoJUnit.rule();

  @Mock private PublicKeyRepository repository;

  @Rule public HttpServerTestRule server = new HttpServerTestRule();

  @Before
  public void setUp() {
    server.setHandler(new PublicKeyReadHandler(repository));
  }

  @Test
  public void returnsOkOnCorrectRequest() throws IOException {
    when(repository.lookupKey(DEVICE_ID1)).thenReturn(ImmutableList.of(PUBLIC_KEY_PEM));
    HttpURLConnection request = getRequestWithQuery("?device-id=my-robot1");

    sendRequest(request);

    assertEquals(getBody(request), PUBLIC_KEY_PEM.value());
  }

  @Test
  public void returnsEmptyResponseIfDeviceNotFound() throws IOException {
    when(repository.lookupKey(DEVICE_ID1)).thenReturn(ImmutableList.of());
    HttpURLConnection request = getRequestWithQuery("?device-id=my-robot1");

    sendRequest(request);

    assertThat(request.getResponseCode(), is(HttpURLConnection.HTTP_OK));
    assertEquals(getBody(request), "");
  }

  @Test
  public void returnsErrorOnNonPostRequest() throws IOException {
    HttpURLConnection request = getRequestWithQuery("?device-id=my-robot1");
    request.setRequestMethod("POST");

    sendRequest(request);

    assertThat(request.getResponseCode(), is(HttpURLConnection.HTTP_BAD_REQUEST));
    assertThat(getBody(request), containsString("This path only supports GET requests"));
  }

  @Test
  @Parameters({"", "?foo=bar", "?device-id"})
  public void returnsAllKeysIfDeviceIdNotSpecified(String query) throws IOException {
    when(repository.listDevices()).thenReturn(ImmutableList.of(DEVICE_ID1));
    when(repository.lookupKey(DEVICE_ID1)).thenReturn(ImmutableList.of(PUBLIC_KEY_PEM));
    HttpURLConnection request = getRequestWithQuery(query);

    sendRequest(request);

    assertThat(request.getResponseCode(), is(HttpURLConnection.HTTP_OK));
    assertEquals(getBody(request), PUBLIC_KEY_PEM.value());
  }

  @Test
  public void returnsKeysForeachDevice() throws IOException {
    when(repository.listDevices()).thenReturn(ImmutableList.of(DEVICE_ID1, DEVICE_ID2));
    when(repository.lookupKey(DEVICE_ID1)).thenReturn(ImmutableList.of(PUBLIC_KEY_PEM));
    when(repository.lookupKey(DEVICE_ID2)).thenReturn(ImmutableList.of(PUBLIC_KEY_PEM));
    HttpURLConnection request = getRequestWithQuery("");

    sendRequest(request);

    assertThat(request.getResponseCode(), is(HttpURLConnection.HTTP_OK));
    assertEquals(getBody(request), PUBLIC_KEY_PEM.value() + PUBLIC_KEY_PEM.value());
  }

  private static void sendRequest(HttpURLConnection request) throws IOException {
    // Querying any part of the response implicitly sends the request (it should actually be
    // sufficient to call URLConnction#connect, but that doesn't seem to work here).
    request.getResponseCode();
  }

  private HttpURLConnection getRequestWithQuery(String query) throws IOException {
    URL url = new URL(server.urlWithPathToHandler() + query);
    HttpURLConnection request = (HttpURLConnection) url.openConnection();
    request.setConnectTimeout(1000);
    request.setReadTimeout(1000);
    return request;
  }

  private static String getBody(HttpURLConnection request) throws IOException {
    InputStream body =
        request.getResponseCode() == HttpURLConnection.HTTP_OK
            ? request.getInputStream()
            : request.getErrorStream();
    return CharStreams.toString(new InputStreamReader(body, Charsets.UTF_8));
  }
}
