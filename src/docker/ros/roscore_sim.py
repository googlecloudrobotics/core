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
  basedir = os.path.dirname(__file__)
  launch_file = os.path.join(basedir, 'roscore_sim.launch')

  with rosenv.RosEnv() as re:
    re.add_package("com_github_ros_ros_comm/tools/rosgraph")
    re.add_package("com_github_ros_ros_comm/tools/roslaunch")
    re.add_package("com_github_ros_ros_comm/tools/rosout")
    re.add_binary("com_github_ros_ros_comm/rosout", "rosout")
    re.add_to_path("com_github_ros_infrastructure_rospkg")
    re.add_to_path("com_github_ros_ros_comm")

    import roslaunch
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
    launch.start()
    launch.spin()


if __name__ == "__main__":
  main()
