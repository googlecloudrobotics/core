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

from prometheus_client import start_http_server, Counter
import rosenv
import rospy
from std_msgs.msg import String

messages = Counter('messages', 'Messages received')


def callback(data):
  rospy.loginfo(rospy.get_caller_id() + 'i heard %s', data.data)
  messages.inc()


if __name__ == '__main__':
  start_http_server(8000)
  with rosenv.RosEnv() as re:
    re.add_package('com_github_ros_ros_comm/tools/rosgraph')
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', String, callback)
    rospy.spin()
