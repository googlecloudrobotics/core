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

import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpServer;
import java.io.IOException;
import java.net.InetSocketAddress;
import org.junit.rules.ExternalResource;

/**
 * Runs a {@link HttpServer} for use in JUnit tests, starting it before all tests run and shutting
 * it down afterwards.
 *
 * <p>Use for testing {@link HttpHandler} implementations.
 */
public class HttpServerTestRule extends ExternalResource {

  private static final String PATH = "/test";
  private HttpServer server;

  /**
   * Sets the {@link HttpHandler} to be tested.
   *
   * <p>This method is intended to be called from an {@link org.junit.Before}-annotated method or
   * likewise (after the server has been started).
   */
  public void setHandler(HttpHandler handler) {
    server.createContext(PATH, handler);
  }

  /**
   * Returns the URL to which the handler passed via {@link #setHandler(HttpHandler)} is bound.
   *
   * <p>The URL contains protocol, address, port and path. E.g.: "http://localhost:1234/test".
   */
  public String urlWithPathToHandler() {
    return String.format("http://localhost:%d%s", server.getAddress().getPort(), PATH);
  }

  @Override
  protected void before() throws IOException {
    // A port number of zero will let the system pick up an ephemeral port in a bind operation.
    server = HttpServer.create(new InetSocketAddress(0), 0);
    server.setExecutor(null); // creates a default executor
    server.start();
  }

  @Override
  protected void after() {
    server.stop(0);
  }
}
