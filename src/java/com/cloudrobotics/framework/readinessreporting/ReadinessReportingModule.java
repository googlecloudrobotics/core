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

package com.cloudrobotics.framework.readinessreporting;

import com.google.common.util.concurrent.ServiceManager;
import com.sun.net.httpserver.HttpHandler;
import dagger.Module;
import dagger.Provides;
import dagger.multibindings.IntoMap;
import dagger.multibindings.StringKey;
import javax.inject.Provider;

/**
 * Include this module to provide readiness reporting HTTP server on standard port 8080 at
 * "/readiness".
 */
@Module
public class ReadinessReportingModule {

  @Provides
  @IntoMap
  @StringKey("/readiness")
  HttpHandler providesReadinessReportingHandler(Provider<ServiceManager> serviceManager) {
    return new ReadinessReportingHandler(serviceManager);
  }
}
