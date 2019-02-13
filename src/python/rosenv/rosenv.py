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

"""Set up environment for ROS nodes.

ROS nodes expect to find executables and data files in a directory structure
pointed to by ROS_ROOT or on the PATH. This library helps a startup script set
up this environment so that the node can run from `bazel run` or in a container.
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import errno
import os
import shutil
import sys
import tempfile


_EXEC_TEMPLATE = """#!/bin/sh
exec {} "$@"
"""


class RosEnv(object):
  def __init__(self):
    # Try to locate the runfiles directory.
    script_path = os.path.abspath(sys.argv[0])
    if ".runfiles" not in script_path:
      raise NotImplementedError("can't find runfiles in " + script_path)
    self.runfiles = script_path[:script_path.find(".runfiles")] + ".runfiles"

  @staticmethod
  def makedirs(path):
    """Like os.makedirs(), but a no-op if the directory already exists."""
    try:
      os.makedirs(path)
    except OSError as e:
      if e.errno != errno.EEXIST:
        raise

  def add_package(self, path, name=None):
    """Add a package to $ROS_ROOT/share.

    path: path to directory containing package.xml, relative to runfiles
          eg com_github_ros_ros_comm/tools/rosgraph
    name: package name (default: last element of path)
    """
    if not os.path.isdir(os.path.join(self.runfiles, path)):
      raise Exception('not a directory: %r' % path)

    name = name or os.path.basename(path)
    os.symlink(os.path.join(self.runfiles, path),
               os.path.join(self.ros_share, name))

  def add_binary(self, path, package="", name=None):
    """Add a binary to $ROS_ROOT/lib.


    path: path to executable, relative to runfiles
          eg com_github_ros_ros_comm/rosout
    package: package name (default: none)
    name: executable name (default: last element of path)
    """
    name = name or os.path.basename(path)
    self.makedirs(os.path.join(self.ros_lib, package))
    new_path = os.path.join(self.ros_lib, package, name)
    target_path = os.path.join(self.runfiles, path)
    os.symlink(target_path, new_path)

  def add_to_path(self, path):
    """Add a directory to the path.

    This is useful for py_binaries as they must be located inside their runfiles
    director or the stub script will fail.

    path: path to directory, relative to runfiles
          eg com_github_ros_infrastructure_rospkg
    """
    os.environ["PATH"] = "%s:%s" % (
        os.path.join(self.runfiles, path), os.environ["PATH"])

  def __enter__(self):
    # TODO(rodrigoq): consider doing this with Bazel, or at least parameterizing
    # it based on the dependencies.
    self.ros_root = tempfile.mkdtemp(prefix="ros.")
    self.ros_share = os.path.join(self.ros_root, "share")
    self.makedirs(self.ros_share)
    self.ros_lib = os.path.join(self.ros_root, "lib")

    # Create catkin marker so catkin.find_in_workspaces uses ROS_ROOT.
    open(os.path.join(self.ros_root, '.catkin'), 'a').close()

    os.environ["CMAKE_PREFIX_PATH"] = self.ros_root
    os.environ["ROS_PACKAGE_PATH"] = self.ros_share
    os.environ["ROS_ROOT"] = self.ros_root

    # Put log files, test results in /tmp to avoid writing to the read-only file
    # system if used with `bazel test`.
    os.environ["ROS_HOME"] = "/tmp/ros_home"
    self.makedirs(os.environ["ROS_HOME"])

    return self

  def __exit__(self, *unused):
    shutil.rmtree(self.ros_root)
