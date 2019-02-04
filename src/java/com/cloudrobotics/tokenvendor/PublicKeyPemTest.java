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

import static org.hamcrest.CoreMatchers.is;
import static org.junit.Assert.assertThat;
import static org.junit.jupiter.api.Assertions.assertThrows;

import junitparams.JUnitParamsRunner;
import junitparams.Parameters;
import org.junit.Test;
import org.junit.runner.RunWith;

/** Tests of {@link PublicKeyPem}. */
@RunWith(JUnitParamsRunner.class)
public class PublicKeyPemTest {

  private static final String VALID_PUBLIC_KEY_BASE64 =
      "MFwwDQYJKoZIhvcNAQEBBQADSwAwSAJBAID64aLO4Gui+Z3uRL0iu/zO6H+b6/jMGPcf"
          + "Gav4Xk5SS8wNEciSwT80Bbu5p2cBFcTHmnsaVhbgfkaeWefEiGcCAwEAAQ==";

  private static final String VALID_PUBLIC_KEY_BASE64_MULTILINE =
      "MFwwDQYJKoZIhvcNAQEBBQADSwAwSAJBAID64aLO4Gui+Z3uRL0iu/zO6H+b6/jMGPcf\n"
          + "Gav4Xk5SS8wNEciSwT80Bbu5p2cBFcTHmnsaVhbgfkaeWefEiGcCAwEAAQ==";

  private static final String VALID_PUBLIC_KEY_PEM =
      "-----BEGIN PUBLIC KEY-----\n" + VALID_PUBLIC_KEY_BASE64 + "\n-----END PUBLIC KEY-----";

  @Test
  @Parameters(method = "validPublicKeyPemCases")
  public void acceptsValidPublicKeyPem(String validKey) {
    PublicKeyPem result = createKey(validKey);

    assertThat(result.value(), is(validKey));
  }

  @Parameters
  public static Object[] validPublicKeyPemCases() {
    return new Object[] {
      "-----BEGIN PUBLIC KEY-----\n" + VALID_PUBLIC_KEY_BASE64 + "\n-----END PUBLIC KEY-----",
      "-----BEGIN PUBLIC KEY-----\n" + VALID_PUBLIC_KEY_BASE64 + "\n-----END PUBLIC KEY-----\n\n",
      "\n\n-----BEGIN PUBLIC KEY-----\n" + VALID_PUBLIC_KEY_BASE64 + "\n-----END PUBLIC KEY-----",
      "-----BEGIN PUBLIC KEY-----\n"
          + VALID_PUBLIC_KEY_BASE64_MULTILINE
          + "\n-----END PUBLIC KEY-----",
    };
  }

  @Test
  @Parameters(method = "formatCases")
  public void storesFormat(PublicKeyPem.Format format) {
    PublicKeyPem result = PublicKeyPem.of(VALID_PUBLIC_KEY_PEM, format);

    assertThat(result.format(), is(format));
  }

  @Parameters
  public static Object[] formatCases() {
    return PublicKeyPem.Format.values();
  }

  @Test
  public void rejectsIfPreambleAndPostambleMissing() {
    assertThrows(IllegalArgumentException.class, () -> createKey(VALID_PUBLIC_KEY_BASE64));
  }

  @Test
  public void rejectsIfTypeIsNotPublicKey() {
    assertThrows(
        IllegalArgumentException.class,
        () ->
            createKey(
                "-----BEGIN INVALID TYPE-----\n"
                    + VALID_PUBLIC_KEY_BASE64
                    + "\n-----END INVALID TYPE-----"));
  }

  @Test
  public void rejectsIfKeyNotBase64Encoded() {
    assertThrows(
        IllegalArgumentException.class,
        () -> createKey("-----BEGIN PUBLIC KEY-----\ninvalid_key\n-----END PUBLIC KEY-----"));
  }

  @Test
  public void rejectsIfKeyEmpty() {
    assertThrows(
        IllegalArgumentException.class,
        () -> createKey("-----BEGIN PUBLIC KEY-----\n\n-----END PUBLIC KEY-----"));
  }

  private static PublicKeyPem createKey(String key) {
    return PublicKeyPem.of(key, PublicKeyPem.Format.RSA_PEM);
  }
}
