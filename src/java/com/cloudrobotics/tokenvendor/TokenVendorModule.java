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

import com.google.api.client.googleapis.auth.oauth2.GoogleCredential;
import com.google.api.client.googleapis.javanet.GoogleNetHttpTransport;
import com.google.api.client.http.HttpTransport;
import com.google.api.client.json.JsonFactory;
import com.google.api.client.json.jackson2.JacksonFactory;
import com.google.api.services.cloudiot.v1.CloudIot;
import com.google.api.services.iam.v1.Iam;
import com.google.api.services.iamcredentials.v1.IAMCredentials;
import com.google.common.flogger.FluentLogger;
import com.sun.net.httpserver.HttpHandler;
import dagger.Module;
import dagger.Provides;
import dagger.multibindings.IntoMap;
import dagger.multibindings.StringKey;
import java.io.IOException;
import java.time.Clock;
import javax.inject.Named;

@Module
class TokenVendorModule {
  private static final FluentLogger logger = FluentLogger.forEnclosingClass();

  private static final String APPLICATION = "com.cloudrobotics.tokenvendor";

  @Provides
  @IntoMap
  @StringKey("/apis/core.token-vendor/v1/token.oauth2")
  HttpHandler providesTokenVendor(TokenVendorHandler tokenVendor) {
    return tokenVendor;
  }

  @Provides
  @IntoMap
  @StringKey("/apis/core.token-vendor/v1/token.verify")
  HttpHandler providesTokenVerifier(VerificationHandler handler) {
    return handler;
  }

  @Provides
  @IntoMap
  @StringKey("/apis/core.token-vendor/v1/public-key.publish")
  HttpHandler providesKeyPublisher(PublicKeyPublishHandler handler) {
    return handler;
  }

  @Provides
  @IntoMap
  @StringKey("/apis/core.token-vendor/v1/public-key.read")
  HttpHandler providesKeyReader(PublicKeyReadHandler handler) {
    return handler;
  }

  @Provides
  JsonFactory providesJsonFactory() {
    return JacksonFactory.getDefaultInstance();
  }

  @Provides
  HttpTransport providesHttpTransport() {
    try {
      return GoogleNetHttpTransport.newTrustedTransport();
    } catch (Exception e) {
      throw new RuntimeException("no HTTPS transport", e);
    }
  }

  @Provides
  GoogleCredential providesGoogleCredential() {
    try {
      return GoogleCredential.getApplicationDefault();
    } catch (IOException e) {
      throw new RuntimeException("no application default credentials", e);
    }
  }

  @Provides
  @Named("iam.authenticated")
  IAMCredentials providesAuthenticatedIAMCredentials(
      HttpTransport httpTransport, JsonFactory jsonFactory, GoogleCredential credential) {
    return new IAMCredentials.Builder(httpTransport, jsonFactory, credential)
        .setApplicationName(APPLICATION)
        .build();
  }

  @Provides
  @Named("iam.unauthenticated")
  Iam providesUnauthenticatedIam(HttpTransport httpTransport, JsonFactory jsonFactory) {
    return new Iam.Builder(httpTransport, jsonFactory, null)
        .setApplicationName(APPLICATION)
        .build();
  }

  @Provides
  CloudIot providesCloudIot(
      HttpTransport httpTransport, JsonFactory jsonFactory, GoogleCredential credential) {
    return new CloudIot.Builder(httpTransport, jsonFactory, credential)
        .setApplicationName(APPLICATION)
        .build();
  }

  @Provides
  Clock providesClock() {
    return Clock.systemUTC();
  }

  @Provides
  PublicKeyPublisher providePublicKeyPublisher(
      Configuration configuration,
      @Named("CloudIotPublicKeyPublisher") PublicKeyPublisher cloudIotPublisher,
      InMemoryPublicKeyRepository inMemoryPublisher) {
    switch (configuration.keyStore) {
      case CLOUD_IOT:
        return cloudIotPublisher;
      case IN_MEMORY:
        logger.atWarning().log(
            "Using in-memory public key storage. "
                + "Published keys will be lost upon shutdown or restart of TokenVendor.");
        return inMemoryPublisher;
      default:
        throw new AssertionError("Invalid enum value");
    }
  }

  @Provides
  PublicKeyRepository providePublicKeyRepository(
      Configuration configuration,
      @Named("CloudIotPublicKeyRepository") PublicKeyRepository cloudIotRepository,
      InMemoryPublicKeyRepository inMemoryRepository) {
    switch (configuration.keyStore) {
      case CLOUD_IOT:
        return cloudIotRepository;
      case IN_MEMORY:
        return inMemoryRepository;
      default:
        throw new AssertionError("Invalid enum value");
    }
  }
}
