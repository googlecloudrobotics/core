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

package com.cloudrobotics.framework.healthreporting;

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

/** Simple health reporting HTTP handler. */
class HealthReportingHandler implements HttpHandler {

  private static final Map<String, List<String>> HEADERS =
      ImmutableMap.of("Content-Type", ImmutableList.of("text/plain"));

  @Override
  public void handle(HttpExchange exchange) throws IOException {
    Headers responseHeaders = exchange.getResponseHeaders();
    responseHeaders.putAll(HEADERS);
    exchange.sendResponseHeaders(HttpURLConnection.HTTP_OK, 0);

    OutputStream responseBody = exchange.getResponseBody();
    String response = "ok\n";
    responseBody.write(response.getBytes(Charset.forName("UTF-8")));
    responseBody.close();
  }
}
