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

import com.google.api.client.auth.oauth2.TokenErrorResponse;
import com.google.api.client.auth.oauth2.TokenResponse;
import com.google.api.client.json.JsonFactory;
import com.google.common.base.Charsets;
import com.google.common.base.Splitter;
import com.google.common.flogger.FluentLogger;
import com.google.common.io.CharStreams;
import com.google.common.net.HttpHeaders;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URLDecoder;
import java.util.Map;
import javax.inject.Inject;

/**
 * The TokenVendorHandler implements a token exchange handler.
 *
 * <p>It returns a JWT for the configured tokenSource in exchange for a JWT validated by the
 * configured tokenVerifier.
 */
class TokenVendorHandler implements HttpHandler {
  private static final FluentLogger logger = FluentLogger.forEnclosingClass();
  private static final String JWT_GRANT = "urn:ietf:params:oauth:grant-type:jwt-bearer";
  private static final String tokenTypePrefix = "token_type=";

  private final JsonFactory jsonFactory;
  private final CloudIotTokenVerifier tokenVerifier;
  private final TokenSource tokenSource;

  @Inject
  TokenVendorHandler(
      JsonFactory jsonFactory, CloudIotTokenVerifier tokenVerifier, TokenSource tokenSource) {
    this.jsonFactory = jsonFactory;
    this.tokenVerifier = tokenVerifier;
    this.tokenSource = tokenSource;
  }

  @Override
  public void handle(HttpExchange exchange) throws IOException {
    try {
      logger.atInfo().log("Handling request %s", exchange.getRequestURI());
      logger.atInfo().log("Request remote %s", exchange.getRemoteAddress());
      String user = tokenVerifier.verifyToken(getJwt(exchange));
      logger.atInfo().log("JWT is valid");
      TokenResponse token = tokenSource.getAccessToken();
      String response = token.toString();
      logger.atInfo().log("Serializing response");
      exchange.getResponseHeaders().add(HttpHeaders.CONTENT_TYPE, "application/json");
      exchange.sendResponseHeaders(HttpURLConnection.HTTP_OK, 0);
      exchange.getResponseBody().write(response.getBytes(Charsets.UTF_8));
    } catch (Exception e) {
      logger.atSevere().withCause(e).log("Exception while minting token");
      // TODO(swolter): Implement more granular error types when needed.
      TokenErrorResponse error =
          new TokenErrorResponse()
              .setError("unauthorized_client")
              .setErrorDescription(e.getMessage());
      error.setFactory(jsonFactory);
      exchange.getResponseHeaders().add(HttpHeaders.CONTENT_TYPE, "application/json");
      exchange.sendResponseHeaders(HttpURLConnection.HTTP_BAD_REQUEST, 0);
      exchange.getResponseBody().write(error.toString().getBytes(Charsets.UTF_8));
    }
    exchange.getResponseBody().close();
  }

  private String getJwt(HttpExchange exchange) throws IOException {
    String query =
        CharStreams.toString(new InputStreamReader(exchange.getRequestBody(), Charsets.UTF_8));
    Map<String, String> queryParameters =
        Splitter.on('&').trimResults().withKeyValueSeparator("=").split(query);
    if (!queryParameters.containsKey("grant_type")) {
      throw new IOException("missing grant type");
    }

    if (!URLDecoder.decode(queryParameters.get("grant_type")).equals(JWT_GRANT)) {
      throw new IOException("invalid grant type");
    }
    if (!queryParameters.containsKey("assertion")) {
      throw new IOException("no JWT provided");
    }
    // Some JWT generators will use base64, not base64url, encoding and then
    // send the assertion URL-encoded. Make sure to decode it.
    return URLDecoder.decode(queryParameters.get("assertion"));
  }
}
