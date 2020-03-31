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

import com.google.api.client.http.HttpHeaders;
import com.google.api.client.http.HttpResponseException;
import com.google.api.services.iam.v1.Iam;
import com.google.api.services.iam.v1.model.TestIamPermissionsRequest;
import com.google.api.services.iam.v1.model.TestIamPermissionsResponse;
import com.google.common.base.Splitter;
import com.google.common.cache.CacheBuilder;
import com.google.common.cache.CacheLoader;
import com.google.common.cache.LoadingCache;
import com.google.common.collect.ImmutableList;
import com.google.common.flogger.FluentLogger;
import com.sun.net.httpserver.Headers;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.URI;
import java.time.Clock;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import javax.inject.Inject;
import javax.inject.Named;

/**
 * VerificationHandler verifies that the access token of the request has a virtual permission.
 *
 * <p>A virtual permission is the actAs on a service account that's used for ACLing, e.g. on
 * human-acl@${PROJECT}.iam.gserviceaccount.com. On success, the handler returns 200, and
 * 401/403/500 on error. This class is used by nginx as a auth_redirect backend.
 */
final class VerificationHandler implements HttpHandler {
  public static final String X_FORWARDED_ACCESS_TOKEN = "X-Forwarded-Access-Token";
  public static final String X_ORIGINAL_URI = "X-Original-URI";

  private static final FluentLogger logger = FluentLogger.forEnclosingClass();
  private static final List<String> roleAccounts = ImmutableList.of("human-acl", "robot-service");
  private final Iam iam;
  private final Configuration configuration;
  // cache for each roleAccount of token -> validationRequestStatusCode
  private final Map<String, LoadingCache<String, Integer>> cache;

  @Inject Clock clock;

  @Inject
  public VerificationHandler(@Named("iam.unauthenticated") Iam iam, Configuration configuration) {
    this.iam = iam;
    this.configuration = configuration;
    this.cache = new HashMap<String, LoadingCache<String, Integer>>();
    for (String roleAccount : roleAccounts) {
      this.cache.put(
          roleAccount,
          CacheBuilder.newBuilder()
              .maximumSize(1000)
              .expireAfterWrite(5, TimeUnit.MINUTES)
              .build(
                  new CacheLoader<String, Integer>() {
                    @Override
                    public Integer load(String token) {
                      try {
                        logger.atInfo().log("verifying token for %s with IAM", roleAccount);
                        return new Integer(checkAuth(roleAccount, token));
                      } catch (Exception e) {
                        logger.atWarning().log("verify failed: %s", e.getMessage());
                        return new Integer(HttpURLConnection.HTTP_INTERNAL_ERROR);
                      }
                    }
                  }));
    }
  }

  @Override
  public void handle(HttpExchange exchange) throws IOException {
    String token = null;
    int statusCode = HttpURLConnection.HTTP_UNAUTHORIZED;
    try {
      token = getToken(exchange.getRequestHeaders());
      boolean robotsAllowed = getRobotsAllowedParameter(exchange.getRequestURI());
      statusCode = cache.get("human-acl").get(token).intValue();
      if (robotsAllowed && statusCode != HttpURLConnection.HTTP_OK) {
        statusCode = cache.get("robot-service").get(token).intValue();
      }
    } catch (Exception e) {
      logger.atInfo().log("Unable to obtain token: %s", e.getMessage());
    }

    exchange.sendResponseHeaders(statusCode, 0);
    exchange.getResponseBody().close();
  }

  int checkAuth(String roleAccount, String token) throws IOException {
    String serviceAccountEmail =
        String.format("%s@%s.iam.gserviceaccount.com", roleAccount, configuration.projectName);
    String resource =
        String.format(
            "projects/%s/serviceAccounts/%s", configuration.projectName, serviceAccountEmail);
    TestIamPermissionsRequest request = new TestIamPermissionsRequest();
    request.setPermissions(ImmutableList.of("iam.serviceAccounts.actAs"));
    HttpHeaders authHeader = new HttpHeaders();
    authHeader.setAuthorization("Bearer " + token);
    try {
      TestIamPermissionsResponse response =
          iam.projects()
              .serviceAccounts()
              .testIamPermissions(resource, request)
              .setRequestHeaders(authHeader)
              .execute();
      if (!request.getPermissions().equals(response.getPermissions())) {
        logger.atInfo().log(
            "User lacks permission for iam.serviceAccounts.actAs on " + "%s", serviceAccountEmail);
        return HttpURLConnection.HTTP_FORBIDDEN;
      }
      logger.atInfo().log("token is verified");
      return HttpURLConnection.HTTP_OK;
    } catch (HttpResponseException e) {
      if (e.getStatusCode() == HttpURLConnection.HTTP_FORBIDDEN
          || e.getStatusCode() == HttpURLConnection.HTTP_UNAUTHORIZED) {
        logger.atInfo().withCause(e).log(
            "IAM responded with %d when testing %s", e.getStatusCode(), serviceAccountEmail);
        return e.getStatusCode();
      }
      throw e;
    }
  }

  String getToken(Headers headers) throws Exception {
    if (headers.containsKey(X_FORWARDED_ACCESS_TOKEN)) {
      return headers.get(X_FORWARDED_ACCESS_TOKEN).get(0);
    }
    if (headers.containsKey(com.google.common.net.HttpHeaders.AUTHORIZATION)) {
      String authorization = headers.get(com.google.common.net.HttpHeaders.AUTHORIZATION).get(0);
      if (!authorization.startsWith("Bearer ")) {
        throw new Exception("Authorization header misses Bearer prefix");
      }
      return authorization.substring("Bearer ".length());
    }
    if (!headers.containsKey(X_ORIGINAL_URI)) {
      throw new Exception("Missing " + X_ORIGINAL_URI + " header");
    }

    String query = new URI(headers.get(X_ORIGINAL_URI).get(0)).getQuery();
    Map<String, String> queryParameters =
        Splitter.on('&').trimResults().withKeyValueSeparator('=').split(query);
    String token = queryParameters.get("token");
    if (token == null) {
      throw new Exception("Missing token query parameter in " + X_ORIGINAL_URI);
    }
    return token;
  }

  private boolean getRobotsAllowedParameter(URI uri) {
    if (uri.getQuery() == null) {
      return false;
    }
    Map<String, String> queryParameters =
        Splitter.on('&').trimResults().withKeyValueSeparator('=').split(uri.getQuery());
    String robots = queryParameters.get("robots");
    if (robots != null && robots.equals("true")) {
      return true;
    }
    // Allow ?robot= as an alias for ?robots= because it's a common typo.
    String robot = queryParameters.get("robot");
    if (robot != null && robot.equals("true")) {
      return true;
    }
    return false;
  }
}
