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

package com.cloudrobotics.framework;

import com.google.common.flogger.FluentLogger;
import com.google.common.util.concurrent.ServiceManager;
import javax.inject.Inject;

/** A super class for Dagger servers. */
public final class ModularServer {

  private static final FluentLogger logger = FluentLogger.forEnclosingClass();

  private final ServiceManager serviceManager;

  @Inject
  public ModularServer(ServiceManager serviceManager) {
    this.serviceManager = serviceManager;
  }

  public static void main(String[] args, ModularServer server) {
    try {
      server.startAndAwaitHealthy();
      logger.atInfo().log("Server healthy.");
    } catch (Throwable t) {
      t.printStackTrace();
      logger.atSevere().log("Exiting due to exception at top-level");
      System.exit(1);
    }
  }

  public final ServiceManager getServiceManager() {
    return serviceManager;
  }

  public final void startAndAwaitHealthy() {
    getServiceManager().startAsync().awaitHealthy();
  }
}
