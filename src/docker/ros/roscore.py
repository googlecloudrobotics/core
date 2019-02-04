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

import sys

import rosenv


def main():
  with rosenv.RosEnv() as re:
    re.add_package("com_github_ros_ros_comm/tools/rosgraph")
    re.add_package("com_github_ros_ros_comm/tools/roslaunch")
    re.add_package("com_github_ros_ros_comm/tools/rosout")
    re.add_binary("com_github_ros_ros_comm/rosout", "rosout")
    re.add_to_path("com_github_ros_infrastructure_rospkg")
    re.add_to_path("com_github_ros_ros_comm")

    import roslaunch
    roslaunch.main(["roscore", "--core"] + sys.argv[1:])


if __name__ == "__main__":
  main()
