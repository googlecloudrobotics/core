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

package com.cloudrobotics.map;

import com.cloudrobotics.framework.FrameworkModule;
import com.cloudrobotics.framework.ModularServer;
import com.cloudrobotics.framework.healthreporting.HealthReportingModule;
import com.cloudrobotics.framework.readinessreporting.ReadinessReportingModule;
import com.cloudrobotics.framework.stats.SystemStatsModule;
import dagger.Component;
import javax.inject.Singleton;

public class MapServer {

  public static void main(String[] args) {
    MapComponent mapComponent = DaggerMapServer_MapComponent.create();
    ModularServer.main(args, mapComponent.modularServer());
  }

  @Singleton
  @Component(
      modules = {
        FrameworkModule.class,
        MapModule.class,
        HealthReportingModule.class,
        ReadinessReportingModule.class,
        SystemStatsModule.class
      })
  public interface MapComponent {

    ModularServer modularServer();
  }
}
