# The test harness is a Python script that sets up the environment with
# rosenv and then starts rostest with the test launch file, which it gets
# on the command line.

from __future__ import print_function

import os
import sys

import rosenv


def main():
  if len(sys.argv) < 2:
    print('usage: %s <path/to/test/node> <rostest parameters>' % sys.argv[0],
          file=sys.stderr)
    sys.exit(1)

  test_node_path = os.path.abspath(sys.argv[1])
  test_node_dir = os.path.dirname(test_node_path)

  # rostest gets its parameters directly from sys.argv. We have to delete our
  # custom parameter so it doesn't get confused.
  del sys.argv[1]

  with rosenv.RosEnv() as re:
    re.add_package("com_github_ros_ros_comm/tools/rosgraph")
    re.add_package("com_github_ros_ros_comm/tools/roslaunch")
    re.add_package(test_node_dir, "${ROSPKG}")
    re.add_to_path("com_github_ros_catkin")
    re.add_to_path("com_github_ros_infrastructure_rospkg")

    import rostest.rostest_main
    rostest.rostest_main.rostestmain()


if __name__ == "__main__":
  main()
