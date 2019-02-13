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

import com.google.common.collect.ImmutableList;
import com.google.common.collect.ImmutableMap;
import com.sun.net.httpserver.Headers;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import java.io.IOException;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.nio.charset.Charset;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import javax.inject.Provider;

public class OverviewHandler implements HttpHandler {

  private final Provider<Map<String, HttpHandler>> handlers;
  private static final Map<String, List<String>> HEADERS =
      ImmutableMap.of("Content-Type", ImmutableList.of("text/html; charset=utf-8"));
  private static final String PAGE_TEMPLATE =
      "<!DOCTYPE HTML><html><head><title>All HTTP Actions</title></head><body>%s</body></html>";
  private static final String LIST_ENTRY_TEMPLATE = "<a href=\"%s\">%s</a><br/>";

  public OverviewHandler(Provider<Map<String, HttpHandler>> handlers) {
    this.handlers = handlers;
  }

  @Override
  public void handle(HttpExchange exchange) throws IOException {
    Headers responseHeaders = exchange.getResponseHeaders();
    responseHeaders.putAll(HEADERS);
    OutputStream responseBody = exchange.getResponseBody();
    exchange.sendResponseHeaders(HttpURLConnection.HTTP_OK, 0);

    StringBuilder listBuilder = new StringBuilder();
    for (Entry<String, HttpHandler> entry : handlers.get().entrySet()) {
      listBuilder.append(String.format(LIST_ENTRY_TEMPLATE, entry.getKey(), entry.getKey()));
    }

    responseBody.write(
        String.format(PAGE_TEMPLATE, listBuilder.toString()).getBytes(Charset.forName("UTF-8")));
    responseBody.close();
  }
}
