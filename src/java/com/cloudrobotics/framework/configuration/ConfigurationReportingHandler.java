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

package com.cloudrobotics.framework.configuration;

import com.google.common.collect.ImmutableList;
import com.google.common.collect.ImmutableMap;
import com.sun.net.httpserver.Headers;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import java.io.IOException;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.nio.charset.Charset;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.List;
import java.util.Map;
import javax.inject.Provider;

public class ConfigurationReportingHandler implements HttpHandler {

  private static final Map<String, List<String>> HEADERS =
      ImmutableMap.of("Content-Type", ImmutableList.of("text/plain"));
  private static final DateFormat DATE_FORMAT = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss");
  private final Provider<ConfigurationService> configurationServiceProvider;

  ConfigurationReportingHandler(Provider<ConfigurationService> configurationServiceProvider) {
    this.configurationServiceProvider = configurationServiceProvider;
  }

  @Override
  public void handle(HttpExchange exchange) throws IOException {
    ConfigurationService configurationService = configurationServiceProvider.get();

    Headers responseHeaders = exchange.getResponseHeaders();
    responseHeaders.putAll(HEADERS);
    exchange.sendResponseHeaders(HttpURLConnection.HTTP_OK, 0);

    OutputStream responseBody = exchange.getResponseBody();
    String response =
        String.format(
            "Last parsed: %s\n\n", DATE_FORMAT.format(configurationService.getLastParsed()));

    responseBody.write(response.getBytes(Charset.forName("UTF-8")));
    responseBody.close();
  }
}
