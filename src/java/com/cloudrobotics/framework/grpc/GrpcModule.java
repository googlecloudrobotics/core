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

package com.cloudrobotics.framework.grpc;

import com.google.common.util.concurrent.Service;
import dagger.Module;
import dagger.Provides;
import dagger.multibindings.IntoSet;
import io.grpc.BindableService;
import io.grpc.ServerInterceptor;
import java.util.Set;

/**
 * Include this module in a component to expose one or more gRPC services. Provide gRPC service
 * implementations as multibindings for io.grpc.BindableService.
 *
 * <p>Change default gRPC port by using the constructor in the component setup:
 *
 * <pre>{@code
 * ExampleComponent.builder()
 *   .grpcModule(new GrpcModule(12345))
 *   .build();
 * }</pre>
 */
@Module
public class GrpcModule {

  private static final int GRPC_PORT = 50051;
  private final int port;

  public GrpcModule() {
    this.port = GRPC_PORT;
  }

  public GrpcModule(int port) {
    this.port = port;
  }

  @Provides
  @IntoSet
  Service provideGrpcService(Set<ServerInterceptor> interceptors, Set<BindableService> services) {
    return new GrpcService(port, interceptors, services);
  }
}
