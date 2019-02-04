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

package com.cloudrobotics.framework.configuration;

import com.google.protobuf.MessageLite;

public final class MessageUtils {

  private static final String GET_DEFAULT_INSTANCE_MSG =
      "Message class must implement the #getDefaultInstance() static method: ";

  public static <M extends MessageLite> M getDefaultInstance(Class<M> type) {
    try {
      return type.cast(type.getMethod("getDefaultInstance").invoke(null));
    } catch (ReflectiveOperationException | ClassCastException e) {
      throw new IllegalArgumentException(GET_DEFAULT_INSTANCE_MSG + type, e);
    }
  }
}
