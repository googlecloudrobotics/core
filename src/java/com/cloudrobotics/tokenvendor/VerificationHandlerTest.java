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

import static org.hamcrest.core.IsEqual.equalTo;
import static org.junit.Assert.*;

import com.google.api.client.http.LowLevelHttpRequest;
import com.google.api.client.http.LowLevelHttpResponse;
import com.google.api.client.json.jackson2.JacksonFactory;
import com.google.api.client.testing.http.MockHttpTransport;
import com.google.api.client.testing.http.MockLowLevelHttpRequest;
import com.google.api.client.testing.http.MockLowLevelHttpResponse;
import com.google.api.services.iam.v1.Iam;
import com.google.common.net.HttpHeaders;
import com.sun.net.httpserver.Headers;
import java.io.IOException;
import java.net.HttpURLConnection;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

@RunWith(JUnit4.class)
public final class VerificationHandlerTest {
  private Headers headers = new Headers();
  private VerificationHandler handler;
  private String requestUrl;
  private int iamResponseCode;
  private String iamResponseContent;

  @Before
  public void setUp() throws Exception {
    iamResponseCode = 0;
    iamResponseContent = "";
    MockHttpTransport transport =
        new MockHttpTransport() {
          @Override
          public LowLevelHttpRequest buildRequest(String method, String url) throws IOException {
            requestUrl = url;
            return new MockLowLevelHttpRequest() {
              @Override
              public LowLevelHttpResponse execute() throws IOException {
                MockLowLevelHttpResponse response = new MockLowLevelHttpResponse();
                response.setStatusCode(iamResponseCode);
                response.setContentType("application/json");
                response.setContent(iamResponseContent);
                return response;
              }
            };
          }
        };
    Configuration configuration = new Configuration();
    configuration.projectName = "cloud-project";
    Iam iam =
        new Iam.Builder(transport, JacksonFactory.getDefaultInstance(), null)
            .setApplicationName("test")
            .build();
    handler = new VerificationHandler(iam, configuration);
  }

  @Test
  public void getToken_getsOriginalUrl() throws Exception {
    headers.set(VerificationHandler.X_ORIGINAL_URI, "http://foo.bar/baz?token=good");
    assertThat(handler.getToken(headers), equalTo("good"));
  }

  @Test
  public void getToken_getsForwardedToken() throws Exception {
    // Use mixed case header to check case normalization
    headers.set(VerificationHandler.X_FORWARDED_ACCESS_TOKEN, "good");
    assertThat(handler.getToken(headers), equalTo("good"));
  }

  @Test
  public void getToken_getsAuthorization() throws Exception {
    headers.set(HttpHeaders.AUTHORIZATION, "Bearer foo");
    assertThat(handler.getToken(headers), equalTo("foo"));
  }

  @Test(expected = Exception.class)
  public void getToken_handlesInvalidUrlHeader() throws Exception {
    headers.set(VerificationHandler.X_ORIGINAL_URI, "http://://foo.bar");
    handler.getToken(headers);
  }

  @Test(expected = Exception.class)
  public void getToken_handlesMissingToken() throws Exception {
    headers.set(VerificationHandler.X_ORIGINAL_URI, "http://foo.bar/baz?param=true");
    handler.getToken(headers);
  }

  @Test(expected = Exception.class)
  public void getToken_handlesMissingHeader() throws Exception {
    handler.getToken(headers);
  }

  @Test
  public void checkAuth_callsRightUrl() throws IOException {
    iamResponseCode = 200;
    iamResponseContent = "{ \"permissions\": [\"iam.serviceAccounts.actAs\"] }";
    handler.checkAuth("human-acl", "ya29.good");
    assertThat(
        requestUrl,
        equalTo(
            "https://iam.googleapis.com/v1/projects/cloud-project/serviceAccounts/"
                + "human-acl@cloud-project.iam.gserviceaccount.com:testIamPermissions"));
  }

  @Test
  public void checkAuth_handlesSuccess() throws IOException {
    iamResponseCode = 200;
    iamResponseContent = "{ \"permissions\": [\"iam.serviceAccounts.actAs\"] }";
    assertThat(handler.checkAuth("human-acl", "ya29.good"), equalTo(HttpURLConnection.HTTP_OK));
  }

  @Test
  public void checkAuth_handlesMissingPermission() throws IOException {
    iamResponseCode = 200;
    iamResponseContent = "{ \"permissions\": [] }";
    assertThat(
        handler.checkAuth("human-acl", "ya29.bad"), equalTo(HttpURLConnection.HTTP_FORBIDDEN));
  }

  @Test
  public void checkAuth_handlesCheckPermissionDenied() throws IOException {
    iamResponseCode = 403;
    iamResponseContent = "{}";
    assertThat(
        handler.checkAuth("human-acl", "ya29.denied"), equalTo(HttpURLConnection.HTTP_FORBIDDEN));
  }

  @Test(expected = IOException.class)
  public void checkAuth_handlesInvalidToken() throws IOException {
    iamResponseCode = 400;
    iamResponseContent = "{}";
    assertThat(
        handler.checkAuth("human-acl", "ya29.ugly"), equalTo(HttpURLConnection.HTTP_SERVER_ERROR));
  }
}
