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

import com.google.cloud.datastore.Datastore;
import com.google.cloud.datastore.testing.LocalDatastoreHelper;
import com.google.common.flogger.FluentLogger;
import java.io.IOException;
import org.junit.rules.ExternalResource;
import org.threeten.bp.Duration;

/** Datastore helper for unittests. */
public class DatastoreTestRule extends ExternalResource {
  private static final FluentLogger logger = FluentLogger.forEnclosingClass();
  private static final LocalDatastoreHelper datastoreHelper = LocalDatastoreHelper.create();

  public static Datastore getDatastore() {
    /* When the datastore emulator crashes, we'll get
     * java.net.ConnectException: Connection refused (Connection refused)
     * For now restart the emulator to let the other tests run.
     * https://github.com/GoogleCloudPlatform/google-cloud-java/issues/3276
     */
    try {
      datastoreHelper.reset();
    } catch (IOException e) {
      logger.atWarning().withCause(e).log("Failed to reset LocalDatastoreHelper, restarting");
      try {
        datastoreHelper.start();
      } catch (IOException | InterruptedException e2) {
        logger.atSevere().withCause(e2).log("Failed to start LocalDatastoreHelper");
        return null;
      }
    }
    return datastoreHelper.getOptions().getService();
  }

  @Override
  protected void before() throws Throwable {
    datastoreHelper.start();
  }

  @Override
  protected void after() {
    try {
      datastoreHelper.stop(Duration.ofSeconds(10));
    } catch (Exception e) {
      logger.atSevere().withCause(e).log("Failed to stop LocalDatastoreHelper");
    }
  }
}
