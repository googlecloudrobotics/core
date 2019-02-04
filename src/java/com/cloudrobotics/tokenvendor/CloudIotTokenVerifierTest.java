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

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.when;

import com.google.common.collect.ImmutableList;
import com.nimbusds.jose.JOSEException;
import com.nimbusds.jose.JWSAlgorithm;
import com.nimbusds.jose.JWSHeader;
import com.nimbusds.jose.JWSSigner;
import com.nimbusds.jose.PlainHeader;
import com.nimbusds.jose.crypto.ECDSASigner;
import com.nimbusds.jose.crypto.RSASSASigner;
import com.nimbusds.jwt.JWTClaimsSet;
import com.nimbusds.jwt.PlainJWT;
import com.nimbusds.jwt.SignedJWT;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.security.KeyPair;
import java.security.PrivateKey;
import java.security.Security;
import java.security.interfaces.ECPrivateKey;
import java.time.Clock;
import java.time.Instant;
import java.time.ZoneId;
import java.util.Date;
import org.bouncycastle.jce.provider.BouncyCastleProvider;
import org.bouncycastle.openssl.PEMReader;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.ExpectedException;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;
import org.mockito.Mock;
import org.mockito.junit.MockitoJUnit;
import org.mockito.junit.MockitoRule;

@RunWith(JUnit4.class)
public final class CloudIotTokenVerifierTest {

  private static String TESTDATA_DIR = "src/java/com/cloudrobotics/tokenvendor/testdata";
  private static final String RSA_PRIVATE_KEY_FILE = "rsa_private.pem";
  private static final String RSA_PUBLIC_KEY_FILE = "rsa_cert.pem";
  private static final String RSA_OTHER_PUBLIC_KEY_FILE = "rsa_cert_other.pem";
  private static final String EC_PRIVATE_KEY_FILE = "ec_private.pem";
  private static final String EC_PUBLIC_KEY_FILE = "ec_public.pem";

  private static String ISSUER = "robot-name";
  private static Instant NOW = Instant.ofEpochSecond(1518616344);

  @Rule public MockitoRule rule = MockitoJUnit.rule();

  @Rule public ExpectedException thrown = ExpectedException.none();

  private final Clock clock = Clock.fixed(NOW, ZoneId.of("Europe/Rome"));
  @Mock private PublicKeyRepository publicKeyRepository;

  private CloudIotTokenVerifier verifier;

  @Before
  public void setUp() throws IOException {
    Security.addProvider(new BouncyCastleProvider());

    Configuration configuration = new Configuration();
    configuration.projectName = "cloud-project";
    configuration.cloudRegion = "europe-west1";
    configuration.registryName = "my-registry";
    configuration.acceptedAudience = "https://integration-test.cloudrobotics.com";
    verifier = new CloudIotTokenVerifier(configuration, clock, publicKeyRepository);
  }

  private void setStoredPublicKeyForIssuer(String publicKeyFile, PublicKeyPem.Format format)
      throws IOException {
    String publicKey =
        new String(Files.readAllBytes(Paths.get(TESTDATA_DIR, publicKeyFile)), "US-ASCII");
    when(publicKeyRepository.lookupKey(DeviceId.of(ISSUER)))
        .thenReturn(ImmutableList.of(PublicKeyPem.of(publicKey, format)));
  }

  private void clearStoredPublicKeyForIssuer(String issuer) {
    when(publicKeyRepository.lookupKey(DeviceId.of(issuer))).thenReturn(ImmutableList.of());
  }

  private JWTClaimsSet.Builder claimsBuilder() {
    return new JWTClaimsSet.Builder()
        .issuer(ISSUER)
        .audience("https://integration-test.cloudrobotics.com")
        .expirationTime(Date.from(NOW.plusSeconds(60)));
  }

  private static String signedRsaJwt(JWTClaimsSet claims) throws IOException, JOSEException {
    PrivateKey privateKey = readPrivateKeyFromFile(RSA_PRIVATE_KEY_FILE);
    return signedJwt(claims, new RSASSASigner(privateKey), JWSAlgorithm.RS256);
  }

  private static String signedEcJwt(JWTClaimsSet claims) throws IOException, JOSEException {
    PrivateKey privateKey = readPrivateKeyFromFile(EC_PRIVATE_KEY_FILE);
    return signedJwt(claims, new ECDSASigner((ECPrivateKey) privateKey), JWSAlgorithm.ES256);
  }

  private static PrivateKey readPrivateKeyFromFile(String ecPrivateKeyFile) throws IOException {
    KeyPair keyPair =
        (KeyPair)
            new PEMReader(new FileReader(new File(TESTDATA_DIR, ecPrivateKeyFile))).readObject();
    return keyPair.getPrivate();
  }

  private static String signedJwt(JWTClaimsSet claims, JWSSigner signer, JWSAlgorithm algorithm)
      throws JOSEException {
    SignedJWT signedJwt = new SignedJWT(new JWSHeader(algorithm), claims);
    signedJwt.sign(signer);
    return signedJwt.serialize();
  }

  @Test
  public void rejectsBogusToken() throws Exception {
    thrown.expect(InvalidTokenException.class);
    setStoredPublicKeyForIssuer(RSA_PUBLIC_KEY_FILE, PublicKeyPem.Format.RSA_PEM);

    verifier.verifyToken("foo.bar.baz");
  }

  @Test
  public void rejectsUnsignedToken() throws Exception {
    thrown.expect(InvalidTokenException.class);
    setStoredPublicKeyForIssuer(RSA_PUBLIC_KEY_FILE, PublicKeyPem.Format.RSA_PEM);
    PlainJWT plainJWT = new PlainJWT(new PlainHeader(), claimsBuilder().build());

    verifier.verifyToken(plainJWT.serialize());
  }

  @Test
  public void rejectsMissingAudience() throws Exception {
    thrown.expect(InvalidTokenException.class);
    setStoredPublicKeyForIssuer(RSA_PUBLIC_KEY_FILE, PublicKeyPem.Format.RSA_PEM);
    String jwt = signedRsaJwt(claimsBuilder().audience((String) null).build());

    verifier.verifyToken(jwt);
  }

  @Test
  public void rejectsWrongAudience() throws Exception {
    thrown.expect(InvalidTokenException.class);
    setStoredPublicKeyForIssuer(RSA_PUBLIC_KEY_FILE, PublicKeyPem.Format.RSA_PEM);
    String jwt = signedRsaJwt(claimsBuilder().audience("http://example.com").build());

    verifier.verifyToken(jwt);
  }

  @Test
  public void acceptsMultipleAudiences() throws Exception {
    setStoredPublicKeyForIssuer(RSA_PUBLIC_KEY_FILE, PublicKeyPem.Format.RSA_PEM);
    String jwt =
        signedRsaJwt(
            claimsBuilder()
                .audience(
                    ImmutableList.of(
                        "https://integration-test.cloudrobotics.com", "http://example.com"))
                .build());

    verifier.verifyToken(jwt);
  }

  @Test
  public void acceptsQualifiedAudiences() throws Exception {
    setStoredPublicKeyForIssuer(RSA_PUBLIC_KEY_FILE, PublicKeyPem.Format.RSA_PEM);
    String jwt =
        signedRsaJwt(
            claimsBuilder()
                .audience(
                    ImmutableList.of(
                        "https://integration-test.cloudrobotics.com?token_type=access_token"))
                .build());

    verifier.verifyToken(jwt);
  }

  @Test
  public void rejectsMissingExpirationTime() throws Exception {
    thrown.expect(InvalidTokenException.class);
    setStoredPublicKeyForIssuer(RSA_PUBLIC_KEY_FILE, PublicKeyPem.Format.RSA_PEM);
    String jwt = signedRsaJwt(claimsBuilder().expirationTime(null).build());

    verifier.verifyToken(jwt);
  }

  @Test
  public void rejectsPastExpirationTime() throws Exception {
    thrown.expect(InvalidTokenException.class);
    setStoredPublicKeyForIssuer(RSA_PUBLIC_KEY_FILE, PublicKeyPem.Format.RSA_PEM);
    String jwt =
        signedRsaJwt(claimsBuilder().expirationTime(Date.from(NOW.minusSeconds(1))).build());

    verifier.verifyToken(jwt);
  }

  @Test
  public void acceptsPastNotBefore() throws Exception {
    setStoredPublicKeyForIssuer(RSA_PUBLIC_KEY_FILE, PublicKeyPem.Format.RSA_PEM);
    String jwt =
        signedRsaJwt(claimsBuilder().notBeforeTime(Date.from(NOW.minusSeconds(1))).build());

    verifier.verifyToken(jwt);
  }

  @Test
  public void rejectsFutureNotBefore() throws Exception {
    thrown.expect(InvalidTokenException.class);
    setStoredPublicKeyForIssuer(RSA_PUBLIC_KEY_FILE, PublicKeyPem.Format.RSA_PEM);
    String jwt = signedRsaJwt(claimsBuilder().notBeforeTime(Date.from(NOW.plusSeconds(1))).build());

    verifier.verifyToken(jwt);
  }

  @Test
  public void rejectsEmptyIssuer() throws Exception {
    thrown.expect(InvalidTokenException.class);
    setStoredPublicKeyForIssuer(RSA_PUBLIC_KEY_FILE, PublicKeyPem.Format.RSA_PEM);
    String jwt = signedRsaJwt(claimsBuilder().issuer(null).build());

    verifier.verifyToken(jwt);
  }

  @Test
  public void rejectsDeviceForWhichNoValidPublicKeyWasPublished() throws Exception {
    thrown.expect(InvalidTokenException.class);
    thrown.expectMessage("no entry for this device or all keys for this device have expired.");
    clearStoredPublicKeyForIssuer(ISSUER);
    String jwt = signedRsaJwt(claimsBuilder().build());

    verifier.verifyToken(jwt);
  }

  @Test
  public void rejectsKeyNotMatchingSignature() throws Exception {
    thrown.expect(InvalidTokenException.class);
    thrown.expectMessage("signature did not match key");
    setStoredPublicKeyForIssuer(RSA_OTHER_PUBLIC_KEY_FILE, PublicKeyPem.Format.RSA_PEM);
    String jwt = signedRsaJwt(claimsBuilder().build());

    verifier.verifyToken(jwt);
  }

  @Test
  public void acceptsValidRsaToken() throws Exception {
    setStoredPublicKeyForIssuer(RSA_PUBLIC_KEY_FILE, PublicKeyPem.Format.RSA_PEM);
    String jwt = signedRsaJwt(claimsBuilder().build());

    verifier.verifyToken(jwt);
  }

  @Test
  public void acceptsValidEcToken() throws Exception {
    setStoredPublicKeyForIssuer(EC_PUBLIC_KEY_FILE, PublicKeyPem.Format.ES256_PEM);
    String signedJwt = signedEcJwt(claimsBuilder().build());

    verifier.verifyToken(signedJwt);
  }

  @Test
  public void returnsIssuer() throws Exception {
    setStoredPublicKeyForIssuer(RSA_PUBLIC_KEY_FILE, PublicKeyPem.Format.RSA_PEM);
    String jwt = signedRsaJwt(claimsBuilder().build());

    String result = verifier.verifyToken(jwt);

    assertEquals(ISSUER, result);
  }
}
