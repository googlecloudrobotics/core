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
import static org.junit.Assert.assertThat;
import static org.mockito.Mockito.verify;

import com.cloudrobotics.framework.HttpServerTestRule;
import com.google.common.base.Charsets;
import com.google.common.io.CharStreams;
import com.google.common.net.HttpHeaders;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
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

/** Tests of {@link PublicKeyPublishHandler}. */
@RunWith(JUnitParamsRunner.class)
public class PublicKeyPublishHandlerTest {

  private static final String PEM_CONTENT_TYPE = "application/x-pem-file";
  private static final String PUBLIC_KEY_PEM =
      "-----BEGIN PUBLIC KEY-----\n"
          + "MFwwDQYJKoZIhvcNAQEBBQADSwAwSAJBAID64aLO4Gui+Z3uRL0iu/zO6H+b6/jMGPcf"
          + "Gav4Xk5SS8wNEciSwT80Bbu5p2cBFcTHmnsaVhbgfkaeWefEiGcCAwEAAQ=="
          + "\n-----END PUBLIC KEY-----";

  @Rule public MockitoRule rule = MockitoJUnit.rule();

  @Mock private PublicKeyPublisher publicKeyPublisher;

  @Rule public HttpServerTestRule server = new HttpServerTestRule();

  @Before
  public void setUp() {
    server.setHandler(new PublicKeyPublishHandler(publicKeyPublisher));
  }

  @Test
  public void returnsOkOnCorrectRequest() throws IOException {
    HttpURLConnection request = postRequestWithQuery("?device-id=my-robot");
    setRequestBody(request, PEM_CONTENT_TYPE, PUBLIC_KEY_PEM);

    sendRequest(request);

    assertThat(request.getResponseCode(), is(HttpURLConnection.HTTP_OK));
  }

  @Test
  public void publishesKeyOnCorrectRequest() throws IOException {
    HttpURLConnection request = postRequestWithQuery("?device-id=my-robot");
    setRequestBody(request, PEM_CONTENT_TYPE, PUBLIC_KEY_PEM);

    sendRequest(request);

    verify(publicKeyPublisher)
        .publishKey(
            DeviceId.of("my-robot"), PublicKeyPem.of(PUBLIC_KEY_PEM, PublicKeyPem.Format.RSA_PEM));
  }

  @Test
  public void returnsErrorOnNonPostRequest() throws IOException {
    HttpURLConnection request = requestWithQuery("?device-id=my-robot");
    request.setRequestMethod("GET");

    sendRequest(request);

    assertThat(request.getResponseCode(), is(HttpURLConnection.HTTP_BAD_REQUEST));
    assertThat(getBody(request), containsString("This path only supports POST requests"));
  }

  @Test
  @Parameters({"", "?foo=bar", "?device-id"})
  public void returnsErrorIfDeviceIdNotSpecified(String query) throws IOException {
    HttpURLConnection request = postRequestWithQuery(query);
    setRequestBody(request, PEM_CONTENT_TYPE, PUBLIC_KEY_PEM);

    sendRequest(request);

    assertThat(request.getResponseCode(), is(HttpURLConnection.HTTP_BAD_REQUEST));
    assertThat(getBody(request), containsString("Missing query parameter: device-id"));
  }

  @Test
  public void returnsErrorIfRequestHasWrongContentType() throws IOException {
    HttpURLConnection request = postRequestWithQuery("?device-id=my-robot");
    setRequestBody(request, "application/json", PUBLIC_KEY_PEM);

    sendRequest(request);

    assertThat(request.getResponseCode(), is(HttpURLConnection.HTTP_BAD_REQUEST));
    assertThat(
        getBody(request), containsString("Expected content type \"application/x-pem-file\""));
  }

  @Test
  public void returnsErrorIfRequestHasInvalidPublicKey() throws IOException {
    HttpURLConnection request = postRequestWithQuery("?device-id=my-robot");
    setRequestBody(request, PEM_CONTENT_TYPE, "Not a public key");

    sendRequest(request);

    assertThat(request.getResponseCode(), is(HttpURLConnection.HTTP_BAD_REQUEST));
    assertThat(getBody(request), containsString("has to be specified in PEM format"));
  }

  private static void sendRequest(HttpURLConnection request) throws IOException {
    // Querying any part of the response implicitly sends the request (it should actually be
    // sufficient to call URLConnction#connect, but that doesn't seem to work here).
    request.getResponseCode();
  }

  private HttpURLConnection postRequestWithQuery(String query) throws IOException {
    HttpURLConnection request = requestWithQuery(query);
    request.setDoOutput(true); // Implicitly sets the method to POST
    return request;
  }

  private HttpURLConnection requestWithQuery(String query) throws IOException {
    URL url = new URL(server.urlWithPathToHandler() + query);
    HttpURLConnection request = (HttpURLConnection) url.openConnection();
    request.setConnectTimeout(1000);
    request.setReadTimeout(1000);
    return request;
  }

  private static void setRequestBody(HttpURLConnection request, String contentType, String body)
      throws IOException {
    request.setRequestProperty(HttpHeaders.CONTENT_TYPE, contentType);
    OutputStream out = request.getOutputStream();
    out.write(body.getBytes(Charsets.UTF_8));
    out.flush();
    out.close();
  }

  private static String getBody(HttpURLConnection request) throws IOException {
    InputStream body =
        request.getResponseCode() == HttpURLConnection.HTTP_OK
            ? request.getInputStream()
            : request.getErrorStream();
    return CharStreams.toString(new InputStreamReader(body, Charsets.UTF_8));
  }
}
