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

import os

import rosenv


def main():
  with rosenv.RosEnv() as re:
    re.add_package("com_github_robotwebtools_rosbridge_suite/rosapi")
    re.add_package("com_github_robotwebtools_rosbridge_suite/rosbridge_server")
    re.add_package(
        "com_github_robotwebtools_tf2_web_republisher",
        "tf2_web_republisher")
    re.add_package("com_github_ros_common_msgs/actionlib_msgs")
    re.add_package("com_github_ros_common_msgs/nav_msgs")
    re.add_package("com_github_ros_common_msgs/sensor_msgs")
    re.add_package("com_github_ros_planning_navigation_msgs/move_base_msgs")
    re.add_package("com_github_ros_ros_comm/tools/rosgraph")
    re.add_package("com_github_ros_ros_comm/tools/roslaunch")
    re.add_package("com_github_ros_ros_comm/tools/rosnode")
    re.add_to_path("com_github_ros_catkin")
    re.add_to_path("com_github_ros_infrastructure_rospkg")
    re.add_binary(
        "com_github_robotwebtools_rosbridge_suite/rosapi_node",
        "rosapi")
    re.add_binary(
        "com_github_robotwebtools_rosbridge_suite/rosbridge_websocket",
        "rosbridge_server")

    launch = os.path.join(
        re.runfiles, "cloud_robotics/src/docker/ros_websocket/websocket.launch")

    import roslaunch
    roslaunch.main(["roslaunch", "--wait", launch])


if __name__ == "__main__":
  main()
