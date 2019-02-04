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

package com.cloudrobotics.framework.stats;

import com.google.common.collect.ImmutableList;
import com.google.common.collect.ImmutableMap;
import com.sun.net.httpserver.Headers;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import java.io.File;
import java.io.IOException;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.nio.charset.Charset;
import java.util.List;
import java.util.Map;

public class SystemStatsHandler implements HttpHandler {

  private static final String PAGE_TEMPLATE =
      "<!DOCTYPE HTML><html><head><title>Memory Stats</title></head><body>"
          + "Available processors (cores): %s<br/>"
          + "Free memory (bytes): %s<br/>"
          + "Maximum memory (bytes): %s<br/>"
          + "Total memory available to JVM (bytes): %s<br/>"
          + "Root file systems:<br/>"
          + "%s"
          + "</body></html>";
  private static final String FILE_SYSTEM_TEMPLATE =
      "File system root: %s<br/>"
          + "Total space (bytes): %s<br/>"
          + "Free space (bytes): %s<br/>"
          + "Usable space (bytes): %s<br/>";
  private static final Map<String, List<String>> HEADERS =
      ImmutableMap.of("Content-Type", ImmutableList.of("text/html; charset=utf-8"));

  @Override
  public void handle(HttpExchange exchange) throws IOException {
    Headers responseHeaders = exchange.getResponseHeaders();
    responseHeaders.putAll(HEADERS);
    OutputStream responseBody = exchange.getResponseBody();

    exchange.sendResponseHeaders(HttpURLConnection.HTTP_OK, 0);

    // Get a list of all filesystem roots on this system.
    File[] roots = File.listRoots();

    // For each filesystem root, print some info.
    StringBuilder filesystemListBuilder = new StringBuilder();
    for (File root : roots) {
      filesystemListBuilder.append(
          String.format(
              FILE_SYSTEM_TEMPLATE,
              root.getAbsolutePath(),
              root.getTotalSpace(),
              root.getFreeSpace(),
              root.getUsableSpace()));
    }

    long maxMemory = Runtime.getRuntime().maxMemory();
    responseBody.write(
        String.format(
                PAGE_TEMPLATE,
                Runtime.getRuntime().availableProcessors(),
                Runtime.getRuntime().freeMemory(),
                maxMemory == Long.MAX_VALUE ? "no limit" : maxMemory,
                Runtime.getRuntime().totalMemory(),
                filesystemListBuilder.toString())
            .getBytes(Charset.forName("UTF-8")));
    responseBody.close();
  }
}
