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

"""Forward ROS log messages to cloud systems."""

import datetime
import json
import os
import sys

import diagnostic_msgs.msg
from google.cloud import logging
from google.cloud import pubsub
import rosgraph_msgs.msg
import rospy
from rospy_message_converter import message_converter


def utc_to_rfc3339(timestamp):
  dt = datetime.datetime.utcfromtimestamp(timestamp)
  return dt.isoformat('T') + 'Z'


class StackdriverLogBridge(object):
  """Subscribes to ROS log messages and sends them to a Stackdriver logger."""

  ROS_LEVEL_TO_SEVERITY = {
      rospy.DEBUG: "DEBUG",
      rospy.INFO: "INFO",
      rospy.WARN: "WARNING",
      rospy.ERROR: "ERROR",
      rospy.FATAL: "CRITICAL",
  }

  def __init__(self, log_name):
    log_client = logging.Client()
    # we can't set the resource type, this will default to 'global'
    # https://github.com/GoogleCloudPlatform/google-cloud-python/issues/2673
    self.logger = log_client.logger(log_name)
    self.logger.log_text('Logging initialized.')

    rospy.Subscriber('/rosout_agg', rosgraph_msgs.msg.Log, self.callback)
    rospy.loginfo('forward "rosout_agg" to "%s"' % self.logger.path)

  def callback(self, log_entry):
    labels = {
        'name': log_entry.name,
        'rosTimestamp': utc_to_rfc3339(log_entry.header.stamp.to_time()),
        'topics': ', '.join(log_entry.topics),
        # TODO(rodrigoq): the Python wrapper doesn't seem to let us put this
        # into Cloud Logging's source location fields.
        'file': log_entry.file,
        'function': log_entry.function,
        'line': str(log_entry.line),
    }
    severity = self.ROS_LEVEL_TO_SEVERITY[log_entry.level]
    self.logger.log_text(log_entry.msg, labels=labels, severity=severity)


class PubSubDiagnosticsBridge(object):
  """Subscribes to ROS diagnostics and sends them to a PubSub topic."""

  def __init__(self, topic_name):
    ps_client = pubsub.Client()
    self.topic = ps_client.topic(topic_name)
    if not self.topic.exists():
      self.topic.create()

    rospy.Subscriber('/diagnostics_agg', diagnostic_msgs.msg.DiagnosticArray,
                     self.callback)
    rospy.loginfo('forward "diagnostics_agg" to "%s"' % topic_name)

  def callback(self, log_entry):
    message = message_converter.convert_ros_message_to_dictionary(log_entry)
    self.topic.publish(json.dumps(message).encode('utf-8'))


def main():
  if '--help' in sys.argv:
    print 'usage: %s' % sys.argv[0]
    return

  rospy.init_node('log_bridge', anonymous=True)
  rospy.loginfo('created ros-node')

  robot_name = os.environ['ROBOT_NAME']

  StackdriverLogBridge('roslog-' + robot_name)
  PubSubDiagnosticsBridge('robots.%s.diagnostics' % robot_name)
  rospy.spin()


if __name__ == '__main__':
  main()
