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

import com.google.api.client.auth.oauth2.TokenResponse;
import com.google.api.client.json.JsonFactory;
import com.google.api.services.iamcredentials.v1.IAMCredentials;
import com.google.api.services.iamcredentials.v1.model.GenerateAccessTokenRequest;
import com.google.api.services.iamcredentials.v1.model.GenerateAccessTokenResponse;
import com.google.common.flogger.FluentLogger;
import java.io.IOException;
import java.security.GeneralSecurityException;
import java.time.Clock;
import java.time.Duration;
import java.time.Instant;
import javax.inject.Inject;
import javax.inject.Named;

/**
 * The TokenSource class obtains access tokens for a foreign service account.
 *
 * <p>Normal libraries like GoogleApplicationCredentials and friends obtain credentials for the
 * default account, like the compute service account. This class obtains them for a different
 * service account that the default account can act as.
 */
class TokenSource {
  private static final FluentLogger logger = FluentLogger.forEnclosingClass();

  private final JsonFactory jsonFactory;
  private final Clock clock;
  private final IAMCredentials iam;
  private final Configuration configuration;

  @Inject
  TokenSource(
      JsonFactory jsonFactory,
      Clock clock,
      @Named("iam.authenticated") IAMCredentials iam,
      Configuration configuration) {
    this.jsonFactory = jsonFactory;
    this.clock = clock;
    this.iam = iam;
    this.configuration = configuration;
  }

  TokenResponse getAccessToken() throws IOException, GeneralSecurityException {
    GenerateAccessTokenRequest request = new GenerateAccessTokenRequest();
    request.setScope(configuration.scopes);
    String name =
        String.format(
            "projects/-/serviceAccounts/%s@%s.iam.gserviceaccount.com",
            configuration.robotName, configuration.projectName);
    logger.atInfo().log("Requesting name: %s", name);
    GenerateAccessTokenResponse response =
        iam.projects().serviceAccounts().generateAccessToken(name, request).execute();
    Instant expirationTime = Instant.parse(response.getExpireTime());
    Instant now = clock.instant();
    TokenResponse result = new TokenResponse();
    result.setFactory(jsonFactory);
    result.setTokenType("Bearer");
    result.setAccessToken(response.getAccessToken());
    result.setExpiresInSeconds(Duration.between(now, expirationTime).getSeconds());
    result.setScope(String.join(" ", configuration.scopes));
    logger.atInfo().log(
        "Got an access token from IAM, expiring in %d seconds", result.getExpiresInSeconds());
    return result;
  }
}
