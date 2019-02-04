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

package com.cloudrobotics.framework.readinessreporting;

import com.google.common.collect.ImmutableList;
import com.google.common.collect.ImmutableMap;
import com.google.common.util.concurrent.Service;
import com.google.common.util.concurrent.ServiceManager;
import com.sun.net.httpserver.Headers;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import java.io.IOException;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.nio.charset.Charset;
import java.util.List;
import java.util.Map;
import javax.inject.Provider;

public class ReadinessReportingHandler implements HttpHandler {

  private static final String PAGE_TEMPLATE = "server state: %s\n\n%s";

  private static final Map<String, List<String>> HEADERS =
      ImmutableMap.of("Content-Type", ImmutableList.of("text/plain"));

  private final Provider<ServiceManager> serviceManager;

  public ReadinessReportingHandler(Provider<ServiceManager> serviceManager) {
    this.serviceManager = serviceManager;
  }

  @Override
  public void handle(HttpExchange exchange) throws IOException {
    Headers responseHeaders = exchange.getResponseHeaders();
    responseHeaders.putAll(HEADERS);
    OutputStream responseBody = exchange.getResponseBody();

    StringBuilder serviceListBuilder = new StringBuilder();
    for (Service service : serviceManager.get().servicesByState().values()) {
      serviceListBuilder.append(service.toString()).append("\n");
    }

    if (serviceManager.get().isHealthy()) {
      exchange.sendResponseHeaders(HttpURLConnection.HTTP_OK, 0);
      responseBody.write(
          String.format(PAGE_TEMPLATE, "ok", serviceListBuilder.toString())
              .getBytes(Charset.forName("UTF-8")));
    } else {
      exchange.sendResponseHeaders(HttpURLConnection.HTTP_INTERNAL_ERROR, 0);
      responseBody.write(
          String.format(PAGE_TEMPLATE, "not healthy", serviceListBuilder.toString())
              .getBytes(Charset.forName("UTF-8")));
    }

    responseBody.close();
  }
}
