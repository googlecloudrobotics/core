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

import com.cloudrobotics.configuration.Config;
import com.google.common.flogger.FluentLogger;
import com.google.protobuf.GeneratedMessage;
import com.google.protobuf.InvalidProtocolBufferException;
import com.google.protobuf.Message;
import com.google.protobuf.Parser;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.file.FileSystems;
import java.nio.file.Path;
import java.nio.file.StandardWatchEventKinds;
import java.nio.file.WatchEvent;
import java.nio.file.WatchKey;
import java.nio.file.WatchService;
import java.util.Date;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class ConfigurationServiceForProto<M extends Message> extends ConfigurationService {

  private static final FluentLogger logger = FluentLogger.forEnclosingClass();

  private final Class<M> protoClass;
  private final Path filepath;

  private WatchService watchService;
  private WatchKey watchKey;
  private M cachedMessage;
  private Date lastParsed;

  public ConfigurationServiceForProto(
      GeneratedMessage.GeneratedExtension<Config, M> extension, Path filepath) {
    this.protoClass = (Class<M>) extension.getMessageDefaultInstance().getClass();
    this.filepath = filepath;
  }

  /** Upcast WatchEvents. */
  @SuppressWarnings("unchecked")
  static <T> WatchEvent<T> cast(WatchEvent<?> event) {
    return (WatchEvent<T>) event;
  }

  private static <M extends Message> void processEvents(
      ConfigurationServiceForProto<M> configurationService) {
    for (; ; ) {

      // wait for key to be signalled
      WatchKey key;
      try {
        key = configurationService.watchService.take();
      } catch (InterruptedException x) {
        return;
      }

      if (key != configurationService.watchKey) {
        System.err.println("WatchKey not recognized!!");
        continue;
      }

      for (WatchEvent<?> event : key.pollEvents()) {
        WatchEvent.Kind kind = event.kind();

        if (kind == StandardWatchEventKinds.OVERFLOW) {
          continue;
        }

        WatchEvent<Path> ev = cast(event);
        Path name = ev.context();

        if (configurationService.filepath.getFileName().equals(name)) {
          logger.atInfo().log(
              "Parsing proto %s: %s", event.kind().name(), configurationService.filepath);
          M m = configurationService.parseMessage();
          configurationService.updateCache(m);
        }
      }

      // reset key and remove from set if directory no longer accessible
      key.reset();
    }
  }

  /** Returns the parser for the proto class by grabbing it from the default proto instance. */
  @SuppressWarnings("unchecked")
  private Parser<M> getParser() {
    M defaultInstance = MessageUtils.getDefaultInstance(protoClass);
    // MessageLite is not a generic class, which forces us to cast here
    return (Parser<M>) defaultInstance.getParserForType();
  }

  /** Register the given directory with the WatchService */
  private void register(Path directory) throws IOException {
    logger.atInfo().log("Watching %s", directory);
    watchKey =
        directory.register(
            watchService,
            StandardWatchEventKinds.ENTRY_CREATE,
            StandardWatchEventKinds.ENTRY_DELETE,
            StandardWatchEventKinds.ENTRY_MODIFY);
  }

  private M parseMessage() {
    try {
      M m = getParser().parseFrom(new FileInputStream(filepath.toString()));
      logger.atInfo().log("Successfully parsed message from %s: %s", filepath, m);
      return m;
    } catch (InvalidProtocolBufferException | FileNotFoundException e) {
      e.printStackTrace();
    }
    return null;
  }

  private synchronized void updateCache(M m) {
    logger.atInfo().log("Updating %s", protoClass);
    cachedMessage = m;
    lastParsed = new Date();
  }

  public synchronized M getConfiguration() {
    return protoClass.cast(cachedMessage);
  }

  public synchronized Date getLastParsed() {
    return lastParsed;
  }

  public synchronized <T extends Message> T getMessage() {
    if (cachedMessage == null) {
      logger.atSevere().log("cachedMessage is null");
    }
    return (T) cachedMessage;
  }

  @Override
  protected void startUp() throws Exception {
    watchService = FileSystems.getDefault().newWatchService();
    ExecutorService executor = Executors.newSingleThreadExecutor();

    register(filepath.getParent());
    M m = parseMessage();
    updateCache(m);

    executor.submit(
        () -> {
          processEvents(this);
        });
    logger.atInfo().log("Started ConfigurationService.");
  }

  @Override
  protected void shutDown() throws Exception {
    logger.atInfo().log("Shutting down ConfigurationService.");
  }
}
