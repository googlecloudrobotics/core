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

package com.cloudrobotics.framework;

import com.google.common.util.concurrent.Service;
import com.google.common.util.concurrent.ServiceManager;
import com.sun.net.httpserver.HttpHandler;
import dagger.Module;
import dagger.Provides;
import dagger.multibindings.IntoMap;
import dagger.multibindings.IntoSet;
import dagger.multibindings.StringKey;
import java.util.Map;
import java.util.Set;
import javax.inject.Provider;
import javax.inject.Singleton;

@Module
public class FrameworkModule {
  private static final int HTTP_PORT = 9090;

  private final int httpPort;

  public FrameworkModule() {
    this.httpPort = HTTP_PORT;
  }

  public FrameworkModule(int httpPort) {
    this.httpPort = httpPort;
  }

  @Provides
  @Singleton
  static ServiceManager provideServiceManager(Set<Service> services) {
    return new ServiceManager(services);
  }

  @Provides
  @IntoSet
  Service provideHttpService(Map<String, HttpHandler> handlers) {
    return new HttpService(httpPort, handlers);
  }

  @Provides
  @IntoMap
  @StringKey("/")
  HttpHandler providesOverviewHandler(Provider<Map<String, HttpHandler>> handlers) {
    return new OverviewHandler(handlers);
  }
}
