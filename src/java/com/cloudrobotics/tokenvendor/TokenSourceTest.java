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

import static org.junit.Assert.*;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.google.api.client.auth.oauth2.TokenResponse;
import com.google.api.client.http.LowLevelHttpRequest;
import com.google.api.client.http.LowLevelHttpResponse;
import com.google.api.client.json.jackson2.JacksonFactory;
import com.google.api.client.testing.http.MockHttpTransport;
import com.google.api.client.testing.http.MockLowLevelHttpRequest;
import com.google.api.client.testing.http.MockLowLevelHttpResponse;
import com.google.api.services.iamcredentials.v1.IAMCredentials;
import com.google.common.collect.ImmutableList;
import com.google.common.flogger.FluentLogger;
import java.io.IOException;
import java.time.Clock;
import java.time.Instant;
import java.time.ZoneId;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

@RunWith(JUnit4.class)
public class TokenSourceTest {
  private static final String ROBOT = "robot-service@cloud-project.iam.gserviceaccount.com";
  private static final String IAM_API =
      "https://iamcredentials.googleapis.com/v1/projects/-/serviceAccounts";
  private static final FluentLogger logger = FluentLogger.forEnclosingClass();

  private MockHttpTransport transport =
      new MockHttpTransport() {

        @Override
        public LowLevelHttpRequest buildRequest(String method, String url) throws IOException {
          if (method.equals("POST")
              && url.equals(String.format("%s/%s:generateAccessToken", IAM_API, ROBOT))) {
            return new MockLowLevelHttpRequest() {
              @Override
              public LowLevelHttpResponse execute() throws IOException {
                requestBody = getContentAsString();
                MockLowLevelHttpResponse response = new MockLowLevelHttpResponse();
                response.setStatusCode(200);
                response.setContentType("application/json");
                response.setContent(
                    "{ \"accessToken\": \"foo\", \"expireTime\": \"2014-10-02T15:01:23.045123456Z\" }");
                return response;
              }
            };
          }
          throw new IOException(String.format("Unexpected request: %s %s", method, url));
        }
      };

  private TokenSource tokenSource;
  private String requestBody;
  private String tokenRequestBody;

  @Before
  public void setUp() throws Exception {
    Configuration configuration = new Configuration();
    configuration.projectName = "cloud-project";
    configuration.robotName = "robot-service";
    configuration.scopes = ImmutableList.of("email", "http://example.com");
    IAMCredentials iam =
        new IAMCredentials.Builder(transport, JacksonFactory.getDefaultInstance(), null)
            .setApplicationName("test")
            .build();
    tokenSource =
        new TokenSource(
            JacksonFactory.getDefaultInstance(),
            Clock.fixed(Instant.ofEpochSecond(1412258483), ZoneId.of("Europe/Rome")),
            iam,
            configuration);
  }

  @Test
  public void testTokenSource_setsRequestedScope() throws Exception {
    TokenResponse token = tokenSource.getAccessToken();

    JsonNode message = new ObjectMapper().readTree(requestBody);
    assertEquals("email", message.get("scope").get(0).textValue());
    assertEquals("http://example.com", message.get("scope").get(1).textValue());
  }

  @Test
  public void testTokenSource_propagatesToken() throws Exception {
    TokenResponse token = tokenSource.getAccessToken();
    assertEquals("foo", token.getAccessToken());
  }

  @Test
  public void testTokenSource_readsExpirationTime() throws Exception {
    TokenResponse token = tokenSource.getAccessToken();
    assertEquals(3600, token.getExpiresInSeconds().longValue());
  }

  @Test
  public void testTokenSource_serializesResponse() throws Exception {
    // It is pretty easy to break the JSON serialization of the TokenResponse class.
    // Check against a golden string to make sure that the JSON is good.
    TokenResponse token = tokenSource.getAccessToken();
    assertEquals(
        "{\"access_token\":\"foo\",\"expires_in\":3600,\"scope\":\"email http://example.com\",\"token_type\":\"Bearer\"}",
        token.toString());
  }
}
