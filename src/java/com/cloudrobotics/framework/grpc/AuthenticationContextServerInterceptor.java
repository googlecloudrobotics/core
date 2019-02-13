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

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.google.common.flogger.FluentLogger;
import io.grpc.Context;
import io.grpc.Contexts;
import io.grpc.Metadata;
import io.grpc.ServerCall;
import io.grpc.ServerCallHandler;
import io.grpc.ServerInterceptor;
import java.util.Base64;

public class AuthenticationContextServerInterceptor implements ServerInterceptor {
  private static final FluentLogger logger = FluentLogger.forEnclosingClass();

  @Override
  public <ReqT, RespT> ServerCall.Listener<ReqT> interceptCall(
      ServerCall<ReqT, RespT> call,
      final Metadata requestHeaders,
      ServerCallHandler<ReqT, RespT> next) {
    Context current = Context.current();

    String metadata = requestHeaders.get(AuthenticationContext.METADATA_AUTHORIZATION);
    if (metadata != null) {
      current = current.withValue(AuthenticationContext.CONTEXT_AUTHORIZATION, metadata);
      logger.atInfo().log("Stored 'authorization' in context");
    }
    metadata = requestHeaders.get(AuthenticationContext.METADATA_USERINFO);
    if (metadata != null) {
      try {
        byte[] infoMessage = Base64.getDecoder().decode(metadata);
        ObjectMapper mapper = new ObjectMapper(new JsonFactory());
        UserInfo userInfo = mapper.readValue(infoMessage, UserInfo.class);
        current = current.withValue(AuthenticationContext.USER_ID, userInfo.email);
        logger.atInfo().log("Stored user email in context");
      } catch (Exception e) {
        logger.atInfo().log("Unable to decipher X-Endpoint-API-UserInfo header");
      }
    }
    return Contexts.interceptCall(current, call, requestHeaders, next);
  }

  private static class UserInfo {
    public String issuer;
    public String id;
    public String email;
  }
}
