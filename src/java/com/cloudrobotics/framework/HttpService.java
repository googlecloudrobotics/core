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

import com.google.common.collect.ImmutableMap;
import com.google.common.util.concurrent.AbstractIdleService;
import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpServer;
import java.net.InetSocketAddress;
import java.util.Map;
import java.util.Map.Entry;

public class HttpService extends AbstractIdleService {

  private static final int SHUTDOWN_TIMEOUT_SECONDS = 5;

  private final ImmutableMap<String, HttpHandler> handlers;
  private final int port;
  private HttpServer httpServer;

  HttpService(int port, Map<String, HttpHandler> handlers) {
    this.handlers = ImmutableMap.copyOf(handlers);
    this.port = port;
  }

  @Override
  protected void startUp() throws Exception {
    httpServer = HttpServer.create(new InetSocketAddress(port), 0);
    for (Entry<String, HttpHandler> entry : handlers.entrySet()) {
      httpServer.createContext(entry.getKey(), entry.getValue());
    }
    httpServer.start();
  }

  @Override
  protected void shutDown() throws Exception {
    httpServer.stop(SHUTDOWN_TIMEOUT_SECONDS);
  }
}
