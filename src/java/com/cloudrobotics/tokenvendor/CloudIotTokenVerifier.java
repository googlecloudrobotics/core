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

import com.google.common.collect.ImmutableList;
import com.google.common.flogger.FluentLogger;
import com.nimbusds.jose.JOSEException;
import com.nimbusds.jose.JWSVerifier;
import com.nimbusds.jose.crypto.ECDSAVerifier;
import com.nimbusds.jose.crypto.RSASSAVerifier;
import com.nimbusds.jwt.JWTClaimsSet;
import com.nimbusds.jwt.SignedJWT;
import java.io.IOException;
import java.io.StringReader;
import java.security.interfaces.ECPublicKey;
import java.security.interfaces.RSAPublicKey;
import java.text.ParseException;
import java.time.Clock;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import javax.inject.Inject;
import org.bouncycastle.openssl.PEMReader;

/**
 * The CloudIotTokenVerifier class validates Google Cloud IoT JWT tokens.
 *
 * <p>It checks syntactic validity, timestamps, and verifies that the tokens are signed by a valid
 * key from the bound {@link PublicKeyRepository}.
 */
class CloudIotTokenVerifier {
  private static final FluentLogger logger = FluentLogger.forEnclosingClass();
  private final Configuration configuration;
  private final Clock clock;
  private final PublicKeyRepository publicKeyRepository;

  @Inject
  public CloudIotTokenVerifier(
      Configuration configuration, Clock clock, PublicKeyRepository publicKeyRepository) {
    this.configuration = configuration;
    this.clock = clock;
    this.publicKeyRepository = publicKeyRepository;
  }

  /**
   * Verifies the correctness of the JWT in tokenString.
   *
   * @throws InvalidTokenException when the token is invalid
   * @throws IOException when Cloud IoT registry is not reachable
   * @returns The token's issuer
   */
  public String verifyToken(String tokenString) throws IOException, InvalidTokenException {
    // All checks according to https://tools.ietf.org/html/rfc7519#section-7.2.
    SignedJWT token = null;
    JWTClaimsSet claims = null;
    try {
      token = SignedJWT.parse(tokenString);
      claims = token.getJWTClaimsSet();
    } catch (ParseException e) {
      throw new InvalidTokenException("token failed to parse", e);
    }
    logger.atInfo().log("checking jwt claims %s", claims);
    String qualifiedAudience =
        String.format("%s?token_type=access_token", configuration.acceptedAudience);
    if (claims.getAudience() == null
        || (!claims.getAudience().contains(configuration.acceptedAudience)
            && !claims.getAudience().contains(qualifiedAudience))) {
      throw new InvalidTokenException(
          "audience must contain "
              + configuration.acceptedAudience
              + " or "
              + qualifiedAudience
              + " instead saw "
              + claims.getAudience());
    }
    // Check for null to avoid accepting tokens with infinite lifetime.
    if (claims.getExpirationTime() == null
        || claims.getExpirationTime().before(Date.from(clock.instant()))) {
      throw new InvalidTokenException("token is expired");
    }
    if (claims.getNotBeforeTime() != null
        && claims.getNotBeforeTime().after(Date.from(clock.instant()))) {
      throw new InvalidTokenException("not-before time is in the future");
    }
    if (claims.getIssuer() == null) {
      throw new InvalidTokenException("issuer is required");
    }

    DeviceId deviceId = DeviceId.of(claims.getIssuer());
    ImmutableList<PublicKeyPem> publicKeys = publicKeyRepository.lookupKey(deviceId);

    if (publicKeys.isEmpty()) {
      throw new InvalidTokenException(
          String.format(
              "No valid key for device %s found. Either there was no entry "
                  + "for this device or all keys for this device have expired.",
              deviceId.value()));
    }

    List<String> reasons = new ArrayList<String>();
    for (PublicKeyPem publicKey : publicKeys) {
      try {
        verifyCredential(token, publicKey);
        return claims.getIssuer();
      } catch (InvalidTokenException e) {
        reasons.add(e.getMessage());
      }
    }
    throw new InvalidTokenException(
        String.format(
            "No usable credentials for device %s : %s",
            deviceId.value(), String.join(", ", reasons)));
  }

  private void verifyCredential(SignedJWT token, PublicKeyPem publicKey)
      throws InvalidTokenException {
    Object key;
    try {
      key = new PEMReader(new StringReader(publicKey.value())).readObject();
    } catch (IOException e) {
      throw new InvalidTokenException("invalid key in registry", e);
    }
    if (key == null) {
      throw new InvalidTokenException("invalid key in registry");
    }
    JWSVerifier verifier = null;
    try {
      switch (publicKey.format()) {
        case RSA_PEM:
          verifier = new RSASSAVerifier((RSAPublicKey) key);
          break;
        case ES256_PEM:
          verifier = new ECDSAVerifier((ECPublicKey) key);
          break;
      }
    } catch (JOSEException e) {
      throw new RuntimeException("key factory cannot handle key", e);
    }
    if (verifier == null) {
      throw new InvalidTokenException("unsupported key format " + publicKey.format().toString());
    }
    try {
      if (!token.verify(verifier)) {
        throw new InvalidTokenException("signature did not match key");
      }
    } catch (JOSEException e) {
      throw new InvalidTokenException("verification failed", e);
    }
  }
}
