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

"""Send and receive opaque messages on ROS topics."""

import rospy


class Publisher(object):
  """Publishes messages without needing to know the type.

  The publisher is lazily created when the user publishes the first message.
  The user must specify the type and md5sum of the message when publishing.
  """

  def __init__(self, topic, queue_size, latch=False):
    self.topic = topic
    self.queue_size = queue_size
    self.latch = latch
    self._publisher = None
    self._data_class = None

  def _create_data_class(self, type, md5sum):
    class DataClass(rospy.AnyMsg):
      _type = type
      _md5sum = md5sum

      def __init__(self, buff):
        self._buff = buff

    return DataClass

  def publish(self, type, md5sum, buff):
    if (self._data_class is None or self._data_class._type != type or
            self._data_class._md5sum != md5sum):
      self._data_class = self._create_data_class(type, md5sum)
      self._publisher = rospy.Publisher(
          self.topic, self._data_class, queue_size=self.queue_size,
          latch=self.latch)

    self._publisher.publish(self._data_class(buff))

# TODO(rodrigoq): provide a way to discover type/md5sum before the first
# message, so that the remote subscriber has a chance to connect before the
# first message is published. Otherwise, the remote subscriber doesn't receive
# the first message. (or maybe it should latch the first message?)


class Subscriber(rospy.Subscriber):
  """Subscribes to messages without needing to know the type.

  The callback receives an instance of rospy.AnyMsg. The type and md5sum can be
  retrieved from the _connection_header dict in the message.
  """

  def __init__(self, topic, callback):
    rospy.Subscriber.__init__(self, topic, rospy.AnyMsg, callback)


if __name__ == '__main__':
  # Test code: forward messages between two topics.
  rospy.init_node('forward', anonymous=True)

  pub = Publisher('chatter2', 10)

  def callback(message):
    pub.publish(
        message._connection_header['type'],
        message._connection_header['md5sum'],
        message._buff)

  sub = Subscriber('chatter', callback)

  rospy.spin()
