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

package com.cloudrobotics.framework.healthreporting;

import com.sun.net.httpserver.HttpHandler;
import dagger.Module;
import dagger.Provides;
import dagger.multibindings.IntoMap;
import dagger.multibindings.StringKey;

/** Include this module to provide a simple health reporting HTTP server on standard port 9090. */
@Module
public class HealthReportingModule {

  @Provides
  @IntoMap
  @StringKey("/health")
  HttpHandler providesHealthReportingHandler() {
    return new HealthReportingHandler();
  }
}
