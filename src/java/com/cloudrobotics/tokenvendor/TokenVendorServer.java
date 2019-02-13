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

import com.beust.jcommander.JCommander;
import com.cloudrobotics.framework.ClockModule;
import com.cloudrobotics.framework.FrameworkModule;
import com.cloudrobotics.framework.ModularServer;
import com.cloudrobotics.framework.healthreporting.HealthReportingModule;
import com.cloudrobotics.framework.readinessreporting.ReadinessReportingModule;
import com.cloudrobotics.framework.stats.SystemStatsModule;
import com.cloudrobotics.tokenvendor.cloudiot.CloudIotModule;
import dagger.BindsInstance;
import dagger.Component;
import java.security.Security;
import javax.inject.Singleton;
import org.bouncycastle.jce.provider.BouncyCastleProvider;

public class TokenVendorServer {

  public static void main(String[] args) throws Exception {
    Security.addProvider(new BouncyCastleProvider());

    Configuration configuration = new Configuration();
    JCommander.newBuilder().addObject(configuration).build().parse(args);

    TokenVendorComponent component =
        DaggerTokenVendorServer_TokenVendorComponent.builder()
            .setConfiguration(configuration)
            .build();
    ModularServer.main(args, component.modularServer());
  }

  @Singleton
  @Component(
      modules = {
        FrameworkModule.class,
        ClockModule.class,
        TokenVendorModule.class,
        HealthReportingModule.class,
        ReadinessReportingModule.class,
        SystemStatsModule.class,
        CloudIotModule.class,
      })
  public interface TokenVendorComponent {
    ModularServer modularServer();

    @Component.Builder
    interface Builder {
      @BindsInstance
      Builder setConfiguration(Configuration configuration);

      TokenVendorComponent build();
    }
  }
}
