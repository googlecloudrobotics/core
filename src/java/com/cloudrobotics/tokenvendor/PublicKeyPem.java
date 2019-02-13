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

import com.google.auto.value.AutoValue;
import java.util.Base64;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Encapsulates a public encryption key encoded in PEM format, i.e., a string value which contains a
 * base64-encoded key wrapped with "-----BEGIN PUBLIC KEY-----\n" and "\n-----END PUBLIC KEY-----".
 */
@AutoValue
public abstract class PublicKeyPem {

  public enum Format {
    RSA_PEM,
    ES256_PEM,
  }

  private static final Pattern PATTERN =
      Pattern.compile(
          "\\s*-----BEGIN PUBLIC KEY-----\n(.+)\n-----END PUBLIC KEY-----\\s*", Pattern.DOTALL);

  /**
   * Creates a new instance from the given string where the contained public key is in the given
   * format.
   *
   * <p>Throws an {@link IllegalArgumentException} if the given string does not start with
   * "-----BEGIN PUBLIC KEY-----\n", does not end with "\n-----END PUBLIC KEY-----" or does not
   * contain a valid base64 encoded value. However, does not verify whether the contained key
   * actually matches the given format. E.g., if the format is RSA_PEM, does not check whether the
   * given key actually is an RSA key.
   */
  public static PublicKeyPem of(String value, Format format) {
    Matcher valueMatcher = PATTERN.matcher(value);
    checkValueMatchesPattern(valueMatcher, value);
    checkIsBase64EncodedValue(valueMatcher.group(1));
    return new AutoValue_PublicKeyPem(value, format);
  }

  private static void checkValueMatchesPattern(Matcher matcher, String value) {
    if (!matcher.matches()) {
      throw new IllegalArgumentException(
          String.format(
              "Public key has to be specified in PEM format, i.e., it has to be of the form "
                  + "\"-----BEGIN PUBLIC KEY-----\\n"
                  + "<base64 encoded public key>"
                  + "\\n-----END PUBLIC KEY-----\". Received value: \"%s\"",
              value));
    }
  }

  private static void checkIsBase64EncodedValue(String encodedKey) {
    try {
      Base64.getDecoder().decode(withoutWhitespaceCharacters(encodedKey));
    } catch (IllegalArgumentException e) {
      throw new IllegalArgumentException(
          String.format("Received public key is not a base64 encoded value: %s", e.getMessage()));
    }
  }

  private static String withoutWhitespaceCharacters(String value) {
    return value.replaceAll("\\s", "");
  }

  /**
   * Returns the encapsulated key in PEM format. In particular, the returned string starts with
   * "-----BEGIN PUBLIC KEY-----\n" and ends with "\n-----END PUBLIC KEY-----".
   */
  public abstract String value();

  public abstract Format format();
}
