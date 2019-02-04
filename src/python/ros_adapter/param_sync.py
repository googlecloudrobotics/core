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

"""Translation and syncing between ROS params and Datastore.

For now, this is a very simple implementation, working with a fixed list of
parameters, and only handling changes in Datastore.
"""

from __future__ import print_function

import os
import sys

from google.cloud import datastore
import rospy


class DatastoreParameters(object):
  """Wraps Datastore with a rosparam-type interface."""

  def __init__(self, project, robot_name, param_names):
    self.robot_name = robot_name
    self.param_names = param_names
    self.ds = datastore.Client(project)
    self.callback = None
    self.old_params = {}

  def datastore_key(self, name):
    return self.ds.key('Param', '%s_%s' % (self.robot_name, name))

  def subscribe(self, callback):
    if self.callback is not None:
      raise ValueError('subscribe() called more than once')
    self.callback = callback

  # TODO(rodrigoq): add retries
  def set_params(self, params):
    with self.ds.transaction():
      for name, value in params.iteritems():
        param_entity = datastore.Entity(self.datastore_key(name),
                                        exclude_from_indexes=['value'])
        param_entity['robot'] = self.robot_name.decode('utf-8')
        param_entity['name'] = name.decode('utf-8')
        param_entity['value'] = value
        self.ds.put(param_entity)

  def poll(self):
    query = self.ds.query(kind='Param')
    query.add_filter('robot', '=', self.robot_name.decode('utf-8'))
    new_params = {e['name']: e['value'] for e in query.fetch()}
    for name in self.param_names:
      if self.old_params.get(name) != new_params.get(name):
        self.callback(name, new_params.get(name))
    self.old_params = new_params


class ParamSync(object):
  """Keeps the ROS parameter server up-to-date.

  Handles changes in the Datastore parameters and updates the corresponding ROS
  parameters.
  """

  def __init__(self, project, robot_name, param_names):
    self.param_names = param_names
    self.cloud_params = DatastoreParameters(project, robot_name, param_names)
    self.initial_sync()
    self.cloud_params.subscribe(self.datastore_callback)

  def initial_sync(self):
    params = {}
    for name in self.param_names:
      try:
        params[name] = bytes(rospy.get_param(name))
        rospy.loginfo('sending parameter %r to Datastore', name)
      except KeyError:
        # We're getting here, if one of the parameters configurred on the
        # adapter (see ros-adapter.yaml) is not set on the robot. This can also
        # happen if the cloud nodes are up, but the roscore of the robot is not
        # conencted.
        pass
    self.cloud_params.set_params(params)

  def datastore_callback(self, name, value):
    if value is not None:
      rospy.loginfo('got new value of %r from Datastore', name)
      rospy.set_param(name, value)
    else:
      rospy.loginfo('parameter %r deleted from Datastore', name)
      rospy.delete_param(name, value)


if __name__ == '__main__':
  # For testing: just run the parameter sync
  rospy.init_node('param_sync_test', anonymous=True)
  ps = ParamSync(
      os.environ['GOOGLE_CLOUD_PROJECT'],
      os.environ['ROBOT_NAME'],
      sys.argv[1:])
  print()
  print('Press enter to end test...')
  print()
  raw_input()
