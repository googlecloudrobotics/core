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

"""Announcement of new adapters.

This allows the other adapters to send out state (latched messages, parameters)
when new adapters come up.

In the future, this may be replaced by the Firestore realtime database, but that
currently doesn't support updates in Python.
"""

import threading
import traceback

import rospy


class Announcer(object):
  """Send an initial announcement and subscribe to other announcements."""

  def __init__(self, robot_name, pubsub_client, network, callback):
    self.callback = callback
    self.topic = pubsub_client.topic('robots.%s.adapter-announce' % robot_name)
    if not self.topic.exists():
      self.topic.create()
    self.topic.publish(network)

    self.subscription = self.topic.subscription('%s.%s' % (
        self.topic.name, network))
    if not self.subscription.exists():
      self.subscription.create()

    self.pull_thread = threading.Thread(target=self.pull_announcements)
    # TODO(rodrigoq): add a way to cleanly stop the thread
    self.pull_thread.daemon = True
    self.pull_thread.start()

  def pull_announcements(self):
    while True:
      try:
        pulled = self.subscription.pull()
        for ack_id, message in pulled:
          self.callback(message.data)
          self.subscription.acknowledge([ack_id])
      except BaseException:
        rospy.logerr('Exception in Announcer pull thread:\n' +
                     ''.join(traceback.format_exc()))
