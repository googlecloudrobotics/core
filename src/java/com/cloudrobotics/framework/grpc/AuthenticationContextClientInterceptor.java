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

package com.cloudrobotics.framework.grpc;

import com.google.common.flogger.FluentLogger;
import io.grpc.Attributes;
import io.grpc.CallCredentials;
import io.grpc.CallOptions;
import io.grpc.Channel;
import io.grpc.ClientCall;
import io.grpc.ClientInterceptor;
import io.grpc.Metadata;
import io.grpc.MethodDescriptor;
import io.grpc.Status;
import java.util.concurrent.Executor;

public class AuthenticationContextClientInterceptor implements ClientInterceptor {
  private static final FluentLogger logger = FluentLogger.forEnclosingClass();

  @Override
  public <ReqT, RespT> ClientCall<ReqT, RespT> interceptCall(
      MethodDescriptor<ReqT, RespT> method, CallOptions callOptions, Channel next) {

    return next.newCall(
        method,
        callOptions.withCallCredentials(
            new CallCredentials() {
              @Override
              public void applyRequestMetadata(
                  MethodDescriptor<?, ?> methodDescriptor,
                  Attributes attributes,
                  Executor executor,
                  MetadataApplier metadataApplier) {
                executor.execute(
                    () -> {
                      try {
                        Metadata headers = new Metadata();

                        String metadata = AuthenticationContext.CONTEXT_AUTHORIZATION.get();
                        if (metadata != null) {
                          headers.put(AuthenticationContext.METADATA_AUTHORIZATION, metadata);
                          logger.atInfo().log("Applied 'authorization' from context");
                        }
                        metadataApplier.apply(headers);
                      } catch (Throwable e) {
                        metadataApplier.fail(Status.UNAUTHENTICATED.withCause(e));
                      }
                    });
              }

              @Override
              public void thisUsesUnstableApi() {}
            }));
  }
}
