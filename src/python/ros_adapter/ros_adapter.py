#!/usr/bin/python
#
# Copyright 2019 The Google Cloud Robotics Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tunnel ROS over Pub/Sub.

Each instance of the ros-adapter talks to the cloud-tunnel instance associated
with the robot. This forms a per-robot PubSub system that bridges ros messages
between the connected parties.

The messages that should be mirrored can be configured per ros-adapter instance
in the ros-adapter.yaml.

The PubSub topics are created as robots.{id}.ros.{rostopic}.
"""

from __future__ import division

import argparse
import cStringIO
import json
import os
import Queue
import threading
import time
import traceback
import urllib2
import yaml

from google.cloud import pubsub
from google.cloud import _helpers

import grpc
import prometheus_client
import rospy

import announce
import param_sync
import shapeshifter


CONFIG_PATH = 'ros-adapter.yaml'
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))


QPS_COUNTER = prometheus_client.Counter(
    'ros_messages_total', 'ROS messages relayed', [
        'direction', 'network', 'topic'])


# TODO(rodrigoq): remove this monkey-patch once either there's a better way to
# do it (see upstream issue:
# https://github.com/GoogleCloudPlatform/google-cloud-python/issues/3594) or
# when we move to another API/transport.
from google.cloud.pubsub import _gax


def make_secure_channel(credentials, user_agent, host, extra_options=()):
  if host == "pubsub.googleapis.com":
    host = os.environ['CLOUD_ROBOTICS_DOMAIN']
  MAX_MESSAGE_BYTES = 100 * 1024 * 1024
  options = (
      ('grpc.max_message_length', MAX_MESSAGE_BYTES),
      ('grpc.max_send_message_length', MAX_MESSAGE_BYTES),
      ('grpc.max_receive_message_length', MAX_MESSAGE_BYTES),
  ) + extra_options
  return _helpers.make_secure_channel(
      credentials, user_agent, host, extra_options)


_gax.make_secure_channel = make_secure_channel


# TODO(rodrigoq): allow live config changes - config in Firestore?
class Config(object):
  def __init__(self, path):
    with open(path, 'r') as f:
      config_dict = yaml.safe_load(f)

    self.topics = config_dict['topics']
    self.params = config_dict['params']


def pubsub_topic_name(robot_name, ros_topic):
  # TODO(rodrigoq): check length. PubSub resource name is 255 chars max.
  # TODO(rodrigoq): use `network` in the topic name so we don't subscribe to
  # our own messages. This might also help the latency issue.
  return 'robots.%s.ros.%s' % (robot_name, ros_topic.replace('/', '.'))


def pubsub_subscription_name(topic_name, network):
  # TODO(rodrigoq): check length. PubSub resource name is 255 chars max.
  return '%s.%s' % (topic_name, network)


class _MaxHzFilter(object):
  def __init__(self, max_hz):
    if max_hz:
      self.period = 1 / max_hz
    else:
      self.period = 0
    self._next_msg = 0

  def pulse(self):
    if not self.period:
      return True

    now = time.time()
    if self._next_msg < now:
      self._next_msg = now + self.period
      return True


class TopicAdapter(object):
  """Adapts messages on one topic:
    - subscribes to the ROS topic and publishes messages to PubSub
    - subscribes to the PubSub topic and publishes messages to ROS
    (filtering out messages we published to avoid loops)
  """

  # Max number of messages in a Publish() RPC. 10 allows ~300 messages/s when
  # RTT = 30ms.
  PUBLISH_MAX_MESSAGES = 10

  # Max number of messages returned by a Pull() RPC. 2x PUBLISH_MAX_MESSAGES
  # because Acknowledge() means pulling requires 2x as many calls.
  PULL_MAX_MESSAGES = 20
  ROS_QUEUE_SIZE = 20

  # When dropping messages, log a warning at most this often.
  WARN_THROTTLE_SECONDS = 1

  def __init__(
          self,
          ros_adapter,
          robot_name,
          pubsub_client,
          network,
          ros_topic,
          params):
    self.ros_adapter = ros_adapter
    self.ros_topic = ros_topic
    self.network = network

    self.latch = params.get('latch', False)
    self.max_hz_filter = _MaxHzFilter(params.get('max_hz', 0))

    self.latched_msg = None

    self.topic = pubsub_client.topic(pubsub_topic_name(robot_name, ros_topic))
    if not self.topic.exists():
      self.topic.create()

    self.subscription = self.topic.subscription(
        pubsub_subscription_name(self.topic.name, network))
    if not self.subscription.exists():
      self.subscription.create()

    self.publish_queue = Queue.Queue()

    # TODO(rodrigoq): add a way to unsubscribe on closing the app
    rospy.loginfo('creating ROS subscriber for %s...', ros_topic)
    self.ros_subscriber = shapeshifter.Subscriber(ros_topic, self.callback)
    # TODO(rodrigoq): we always use latched publishers because we register
    # lazily on the first message, meaning that subscribers otherwise miss the
    # first message. This could have unintended effects, so it would be better
    # to have some sort of delay/timeout on the first message.
    self.ros_publisher = shapeshifter.Publisher(ros_topic, self.ROS_QUEUE_SIZE,
                                                latch=True)

    self.pull_thread = threading.Thread(target=self.pull_pubsub_messages)
    # TODO(rodrigoq): add a way to cleanly stop the thread
    self.pull_thread.daemon = True
    self.pull_thread.start()

    self.publish_thread = threading.Thread(target=self.publish_pubsub_messages)
    # TODO(rodrigoq): add a way to cleanly stop the thread
    self.publish_thread.daemon = True
    self.publish_thread.start()

  # TODO(rodrigoq): add retries. should probably copy the notifier as I'm
  # rewriting it here...
  def pull_pubsub_messages(self):
    while True:
      try:
        rospy.logdebug('starting pull on %s...', self.ros_topic)
        pulled = self.subscription.pull(max_messages=self.PULL_MAX_MESSAGES)
        if pulled:
          rospy.logdebug(
              'pulled %d messages on %s',
              len(pulled),
              self.ros_topic)

        for ack_id, message in pulled:
          self.publish_to_ros(message)
        self.subscription.acknowledge([i for i, _ in pulled])
      except BaseException:
        rospy.logerr('Exception in Pub/Sub pull thread:\n' +
                     ''.join(traceback.format_exc()))

  def publish_pubsub_messages(self):
    while True:
      try:
        to_publish = [self.publish_queue.get()]
        try:
          while True:
            to_publish.append(self.publish_queue.get_nowait())
        except Queue.Empty:
          pass

        with self.topic.batch() as batch:
          for msg in to_publish:
            batch.publish(msg)
        rospy.logdebug('published %d messages to Pub/Sub on %s',
                       len(to_publish), self.ros_topic)

      except BaseException:
        rospy.logerr('Exception in Pub/Sub publish thread:\n' +
                     ''.join(traceback.format_exc()))

  def publish_to_ros(self, pubsub_message):
    message = json.loads(pubsub_message.data.decode('utf-8'))
    if message['source'] == self.network:
      return
    self.ros_publisher.publish(
        message['type'],
        message['md5sum'],
        message['buff'].encode('latin-1'))
    QPS_COUNTER.labels(
        direction="out",
        network=self.network,
        topic=self.ros_topic).inc()

  def callback(self, message):
    if message._connection_header['callerid'] == rospy.get_caller_id():
      return
    if not self.max_hz_filter.pulse():
      return

    # Extract metadata from the rospy.AnyMsg instance. This is required to
    # advertise the topic correctly on the other end.
    type = message._connection_header['type']
    md5sum = message._connection_header['md5sum']

    if hasattr(message, '_buff'):
      buff = message._buff
    else:
      # Some topics (eg clock) already have subscribers created by rospy, which
      # means that we have to use the data class specified by rospy instead of
      # our own rospy.AnyMsg.
      sio = cStringIO.StringIO()
      message.serialize(sio)
      buff = sio.getvalue()

    # TODO(rodrigoq): use proto instead of json
    serialized = json.dumps({
        'type': type,
        'md5sum': md5sum,
        'source': self.network,
        'buff': buff.decode('latin-1'),
    }).encode('utf-8')

    self.latched_msg = serialized
    self.add_to_queue(serialized)
    QPS_COUNTER.labels(
        direction="in",
        network=self.network,
        topic=self.ros_topic).inc()

  def on_announcement(self):
    if self.latched_msg:
      self.add_to_queue(self.latched_msg)

  def add_to_queue(self, serialized):
    self.publish_queue.put_nowait(serialized)
    while self.publish_queue.qsize() > self.PUBLISH_MAX_MESSAGES:
      rospy.logwarn_throttle(
          self.WARN_THROTTLE_SECONDS,
          'dropping message(s?) on %s due to full buffer' % self.ros_topic)
      # drop the oldest message and put the new message again
      try:
        self.publish_queue.get_nowait()
      except Queue.Empty:
        pass


class RosAdapter(object):
  def __init__(self, config, project, robot_name, pubsub_client, network):
    self.config = config
    self.project = project
    self.robot_name = robot_name
    self.pubsub_client = pubsub_client
    self.network = network

    self.announcer = None
    self.topic_adapters = {}

  def start(self):
    for topic, params in self.config.topics.items():
      self.add_topic(topic, params)

    self.param_sync = param_sync.ParamSync(
        self.project, self.robot_name, self.config.params)

    self.announcer = announce.Announcer(
        self.robot_name,
        self.pubsub_client,
        self.network,
        self.on_announcement)

    # Announce to ourself to avoid a race condition between initial publishing
    # and subscribing to announcements.
    self.on_announcement(self.network)

  def add_topic(self, topic, params):
    if topic not in self.topic_adapters:
      self.topic_adapters[topic] = TopicAdapter(
          self, self.robot_name, self.pubsub_client, self.network, topic, params)

  def on_announcement(self, new_network):
    rospy.loginfo('received announcement from %r', new_network)

    for topic_adapter in self.topic_adapters.values():
      topic_adapter.on_announcement()

    self.param_sync.cloud_params.poll()


def main():
  parser = argparse.ArgumentParser(description='Tunnel ROS over Pub/Sub.')
  parser.add_argument('network',
                      help='Unique ID for the network this adapter runs'
                      ' on, eg robot, cloud, dev')
  args = parser.parse_args()

  try:
    prometheus_client.start_http_server(80)
  except BaseException:
    rospy.loginfo('failed to bind prometheus HTTP server to port 80')

  config = Config(os.path.join(SCRIPT_DIR, CONFIG_PATH))

  rospy.loginfo('connecting to ros master ' + os.environ['ROS_MASTER_URI'])
  rospy.init_node('ros_adapter_' + args.network)

  project = os.environ['GOOGLE_CLOUD_PROJECT']
  robot_name = os.environ['ROBOT_NAME']
  pubsub_client = pubsub.Client()

  ros_adapter = RosAdapter(
      config,
      project,
      robot_name,
      pubsub_client,
      args.network)
  ros_adapter.start()

  rospy.loginfo('ros-adapter has started')
  rospy.spin()


if __name__ == '__main__':
  main()
