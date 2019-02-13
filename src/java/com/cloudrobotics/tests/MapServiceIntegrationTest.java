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

package com.cloudrobotics.tests;

import static org.junit.Assert.assertEquals;

import com.google.api.client.googleapis.auth.oauth2.GoogleCredential;
import com.google.common.flogger.FluentLogger;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.Collections;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

@RunWith(JUnit4.class)
public class MapServiceIntegrationTest {
  private static final FluentLogger logger = FluentLogger.forEnclosingClass();
  private static final String domain = System.getProperty("CLOUD_ROBOTICS_DOMAIN");
  private static final String baseURL = "https://" + domain + "/apis/core.map";

  private static final String SCOPE = "https://www.googleapis.com/auth/cloud-platform";

  private HttpURLConnection openConnection(String path) throws Exception {
    URL service = new URL(baseURL + path);
    HttpURLConnection connection = (HttpURLConnection) service.openConnection();

    connection.setReadTimeout(60 * 1000);
    connection.setConnectTimeout(60 * 1000);
    return connection;
  }

  private String getAuthorization() throws Exception {
    GoogleCredential credential =
        GoogleCredential.getApplicationDefault().createScoped(Collections.singleton(SCOPE));
    if (credential.getAccessToken() == null) {
      if (!credential.refreshToken()) {
        logger.atSevere().log("Failed to refresh access token");
      }
    }
    return String.format("Bearer %s", credential.getAccessToken());
  }

  @Test
  public void testHttp_redirectsOrThrows() throws Exception {
    try {
      URL service = new URL("http://" + domain + "/apis/core.map/v1alpha1/maps");
      HttpURLConnection connection = (HttpURLConnection) service.openConnection();

      connection.setFollowRedirects(false);

      // Redirecting from http to https is on by default:
      // https://github.com/kubernetes/ingress-nginx/blob/master/docs/user-guide/nginx-configuration/annotations.md#server-side-https-enforcement-through-redirect
      // 308 is Permanent Redirect
      assertEquals(308, connection.getResponseCode());
      assertEquals(connection.getHeaderField("Location"), baseURL + "/v1alpha1/maps");
    } catch (java.net.ConnectException e) {
      // It's fine if port 80 isn't opened in the first place.
    }
  }

  @Test
  public void testListMapsWithoutCredentials_returnsUnauthorized() throws Exception {
    HttpURLConnection connection = openConnection("/v1alpha1/maps");

    assertEquals(HttpURLConnection.HTTP_UNAUTHORIZED, connection.getResponseCode());
  }

  @Test
  public void testListMapsWithInvalidCredentials_returnsUnauthorized() throws Exception {
    HttpURLConnection connection = openConnection("/v1alpha1/maps");

    connection.setRequestProperty(
        "Authorization", "Bearer ya29.safatgwetuabaswtwqptiuetpaiuiucvbxvcmnv");
    assertEquals(HttpURLConnection.HTTP_UNAUTHORIZED, connection.getResponseCode());
  }

  @Test
  public void testListMaps_hasSuccessStatus() throws Exception {
    HttpURLConnection connection = openConnection("/v1alpha1/maps");

    connection.setRequestProperty("Authorization", getAuthorization());

    assertEquals(HttpURLConnection.HTTP_OK, connection.getResponseCode());
  }
}
