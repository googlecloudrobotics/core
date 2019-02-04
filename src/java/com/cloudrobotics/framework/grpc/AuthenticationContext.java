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

import io.grpc.Context;
import io.grpc.Metadata;

public class AuthenticationContext {
  protected static final Context.Key<String> CONTEXT_AUTHORIZATION = Context.key("authorization");
  protected static final Metadata.Key<String> METADATA_AUTHORIZATION =
      Metadata.Key.of("authorization", Metadata.ASCII_STRING_MARSHALLER);

  public static final Context.Key<String> USER_ID = Context.key("user-id");
  protected static final Metadata.Key<String> METADATA_USERINFO =
      Metadata.Key.of("X-Endpoint-API-UserInfo", Metadata.ASCII_STRING_MARSHALLER);
}
