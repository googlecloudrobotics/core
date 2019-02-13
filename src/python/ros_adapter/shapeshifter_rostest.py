# Copyright 2019 The Cloud Robotics Authors
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

"""Tests for shapeshifter."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import cStringIO
import time
import unittest

import mock
import rospy
import rostest
import std_msgs

import shapeshifter

# ROS queue size to use for publishers in tests. Must be greater than the number
# of messages that will be pending send at any time.
QUEUE_SIZE = 10


class ShapeshifterTest(unittest.TestCase):

  # Better error messages from assertion methods.
  longMessage = True

  def waitForCall(self, callback, timeout=1.0):
    """Waits for the callback to be called.

    Raises an AssertionError if the callback is not called within the timeout.

    Args:
      timeout: number of seconds to wait (default: 1)

    Returns:
      a tuple (args, kwargs) of the args the callback was called with
    """
    end = time.time() + timeout
    while time.time() < end:
      if callback.call_count > 0:
        return callback.call_args
      time.sleep(0.1)
    raise AssertionError("callback not called after %.1f seconds" % timeout)

  def serializeMsg(self, msg, *args, **kwargs):
    """Instantiate and serialize a ROS message."""
    buff = cStringIO.StringIO()
    msg(*args, **kwargs).serialize(buff)
    return buff.getvalue()

  def testSubscriber_MessageHasConnectionHeader(self):
    """Checks messages received by Subscriber have connection header.

    Publishes a string with rospy.Publisher, receives it with
    shapeshifter.Subscriber, and checks it has _connection_header.
    """
    callback = mock.Mock()
    s = shapeshifter.Subscriber(self.id(), callback)

    p = rospy.Publisher(
        self.id(),
        std_msgs.msg.String,
        queue_size=QUEUE_SIZE,
        latch=True)
    p.publish(std_msgs.msg.String("hello"))

    (message,), _ = self.waitForCall(callback)
    self.assertTrue(hasattr(message, "_connection_header"),
                    "message has no _connection_header attribute")

  def testSubscriber_MessageHasSerializedString(self):
    """Checks messages received by Subscriber have serialized contents.

    Publishes a string with rospy.Publisher, receives it with
    shapeshifter.Subscriber, and checks it contains a serialized string in
    _buff.
    """
    callback = mock.Mock()
    s = shapeshifter.Subscriber(self.id(), callback)

    p = rospy.Publisher(
        self.id(),
        std_msgs.msg.String,
        queue_size=QUEUE_SIZE,
        latch=True)
    p.publish(std_msgs.msg.String("hello"))

    (message,), _ = self.waitForCall(callback)
    ros_string = std_msgs.msg.String()
    ros_string.deserialize(message._buff)
    self.assertEqual(ros_string.data, "hello", "message contains wrong string")

  def testPublisher_MessageHasCorrectString(self):
    """Checks messages published by Publisher have the correct contents.

    Publishes a serialized string with shapeshifter.Publisher, receives it with
    rospy.Subscriber, and checks the original string was correctly deserialized.
    """
    callback = mock.Mock()
    s = rospy.Subscriber(self.id(), std_msgs.msg.String, callback)

    p = shapeshifter.Publisher(self.id(), queue_size=QUEUE_SIZE, latch=True)
    p.publish(std_msgs.msg.String._type, std_msgs.msg.String._md5sum,
              self.serializeMsg(std_msgs.msg.String, "hello"))

    (message,), _ = self.waitForCall(callback)
    self.assertEqual(message.data, "hello", "message contains wrong string")


if __name__ == '__main__':
  rospy.init_node('shapeshifter_rostest')
  rostest.rosrun('ros_adapter', 'shapeshifter_rostest', ShapeshifterTest)
