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

import static java.util.concurrent.TimeUnit.SECONDS;

import dagger.Module;
import dagger.Provides;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import javax.inject.Singleton;

@Module
public class ExecutorModule {

  @Provides
  @Singleton
  ScheduledExecutorService provideScheduledExecutorService() {
    ScheduledExecutorService executor = Executors.newScheduledThreadPool(1);

    // Cleanly shutdown thread pool if we get terminated.
    Runtime.getRuntime()
        .addShutdownHook(
            new Thread(
                () -> {
                  System.out.println("Shutting down thread pool...");
                  executor.shutdown();
                  try {
                    // Await completion of existing tasks. This timeout should be <= the Kubernetes
                    // grace period, which defaults to 30 seconds.
                    if (executor.awaitTermination(10, SECONDS)) {
                      System.out.println("Shut down cleanly.");
                      return;
                    }
                  } catch (InterruptedException e) {
                    // Ignore since we're shutting down anyway.
                  }
                }));

    return executor;
  }
}
