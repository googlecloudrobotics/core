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

import com.google.common.flogger.FluentLogger;
import com.google.common.util.concurrent.AbstractIdleService;
import io.grpc.BindableService;
import io.grpc.Server;
import io.grpc.ServerBuilder;
import io.grpc.ServerInterceptor;
import io.grpc.ServerInterceptors;
import io.grpc.ServerServiceDefinition;
import io.grpc.protobuf.services.ProtoReflectionService;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.concurrent.TimeUnit;
import javax.inject.Qualifier;

/** Starts / Manages the lifecycle of a gRPC server. */
public class GrpcService extends AbstractIdleService {

  private static final int SHUTDOWN_TIMEOUT_SECONDS = 5;

  private static final FluentLogger logger = FluentLogger.forEnclosingClass();
  private final Server server;

  public GrpcService(
      @GrpcPort Integer port,
      Set<ServerInterceptor> interceptors,
      Set<BindableService> grpcServices) {
    ServerBuilder serverBuilder =
        ServerBuilder.forPort(port).addService(ProtoReflectionService.newInstance());
    List<ServerInterceptor> interceptorList = new ArrayList(interceptors);

    for (BindableService grpcService : grpcServices) {
      logger.atInfo().log("Adding service.%s", grpcService.getClass().toString());
      serverBuilder.addService(ServerInterceptors.intercept(grpcService, interceptorList));
    }
    server = serverBuilder.build();
  }

  @Override
  protected void startUp() throws Exception {
    server.start();
    StringBuilder servicesString = new StringBuilder();
    for (ServerServiceDefinition service : server.getImmutableServices()) {
      servicesString.append("\n * ").append(service.getServiceDescriptor().getName());
    }

    logger.atInfo().log(
        "Started GrpcService service on port %s for services:%s", server.getPort(), servicesString);
  }

  @Override
  protected void shutDown() throws Exception {
    logger.atInfo().log("Shutting down GrpcService service on port %s", server.getPort());
    if (server.isShutdown()) {
      return;
    }
    server.shutdown();
    server.awaitTermination(SHUTDOWN_TIMEOUT_SECONDS, TimeUnit.SECONDS);
    if (!server.isTerminated()) {
      logger.atSevere().log("Failed to shutdown gRPC server.");

      // Force shutdown through thread interruption.
      Thread.currentThread().interrupt();
    }
  }

  @Qualifier
  @Retention(RetentionPolicy.RUNTIME)
  public @interface GrpcPort {}
}
