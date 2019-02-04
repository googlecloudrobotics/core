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

import com.beust.jcommander.Parameter;
import com.beust.jcommander.converters.EnumConverter;
import java.util.List;

/** The Configuration class stores configuration strings. */
public class Configuration {
  @Parameter(names = "--project", required = true, description = "The cloud project")
  public String projectName;

  @Parameter(names = "--region", required = true, description = "The cloud region")
  public String cloudRegion;

  @Parameter(names = "--registry", required = true, description = "The cloud IoT registry name")
  public String registryName;

  @Parameter(
      names = "--accepted_audience",
      required = true,
      description = "Endpoint url of the service.")
  public String acceptedAudience;

  @Parameter(
      names = "--service_account",
      required = true,
      description = "Name of the service account to use.")
  public String robotName;

  @Parameter(names = "--scope", required = true, description = "Authentication scopes")
  public List<String> scopes;

  public enum KeyStore {
    IN_MEMORY,
    CLOUD_IOT
  }

  private static final class KeyStoreConverter extends EnumConverter<KeyStore> {
    KeyStoreConverter(String optionName, Class<KeyStore> clazz) {
      super(optionName, clazz);
    }
  }

  @Parameter(
      names = "--key-store",
      required = true,
      description =
          "Values: [IN_MEMORY, CLOUD_IOT]. CLOUD_IOT: keys will be stored and read from "
              + "the Cloud Iot Registry. IN_MEMORY: keys will be stored in-memory and thus "
              + "all stored keys will be lost upon a shutdown or restart of TokenVendor "
              + "(only use this option if a truly persistent key storage is not required, e.g., "
              + "in an integration test.",
      converter = KeyStoreConverter.class)
  public KeyStore keyStore;
}
