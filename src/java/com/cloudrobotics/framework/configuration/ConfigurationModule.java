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

package com.cloudrobotics.framework.configuration;

import com.cloudrobotics.configuration.Config;
import com.google.common.util.concurrent.Service;
import com.google.protobuf.GeneratedMessage;
import com.google.protobuf.Message;
import com.sun.net.httpserver.HttpHandler;
import dagger.Module;
import dagger.Provides;
import dagger.multibindings.IntoMap;
import dagger.multibindings.IntoSet;
import dagger.multibindings.StringKey;
import java.nio.file.Path;
import javax.inject.Provider;
import javax.inject.Singleton;

@Module
public class ConfigurationModule {

  private final ConfigurationService configurationService;

  public ConfigurationModule(ConfigurationService configurationService) {
    this.configurationService = configurationService;
  }

  public static <M extends Message> ConfigurationModule createForExtension(
      GeneratedMessage.GeneratedExtension<Config, M> extension, Path filepath) {
    return new ConfigurationModule(new ConfigurationServiceForProto<>(extension, filepath));
  }

  @Provides
  @Singleton
  ConfigurationService provideConfigurationService() {
    return configurationService;
  }

  @Provides
  @IntoSet
  @Singleton
  Service provideService(ConfigurationService configurationService) {
    return configurationService;
  }

  @Provides
  @IntoMap
  @StringKey("/config")
  HttpHandler providesConfigurationReportingHandler(
      Provider<ConfigurationService> configurationService) {
    return new ConfigurationReportingHandler(configurationService);
  }
}
