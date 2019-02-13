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

package com.cloudrobotics.tokenvendor.cloudiot;

import com.cloudrobotics.tokenvendor.PublicKeyPublisher;
import com.cloudrobotics.tokenvendor.PublicKeyRepository;
import dagger.Binds;
import dagger.Module;
import javax.inject.Named;

/** Provides bindings for the integration with the Cloud IoT Registry. */
@Module
public abstract class CloudIotModule {

  @Binds
  @Named("CloudIotPublicKeyPublisher")
  public abstract PublicKeyPublisher bindPublicKeyPublisher(
      CloudIotPublicKeyRepository cloudIotPublicKeyRepository);

  @Binds
  @Named("CloudIotPublicKeyRepository")
  public abstract PublicKeyRepository bindPublicKeyRepository(
      CloudIotPublicKeyRepository cloudIotPublicKeyRepository);
}
