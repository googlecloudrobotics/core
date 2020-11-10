#!/usr/bin/env bash
#
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

# This script can be run just like the regular dep tool. It copies the Go
# code to a shadow repo against dep can operate as usual and copies the
# resulting Gopkg.toml and Gopkg.lock files to this directory.
# It then stages the changed dependenies in the bazel WORKSPACE for manual cleanup.

set -e

echo "Set NO_TEARDOWN=y for preserve \"kind\" clusters past test failures"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

CLOUD_ROBOTICS_CONTAINER_REGISTRY=${CLOUD_ROBOTICS_CONTAINER_REGISTRY:-$1}
if [[ -z "${CLOUD_ROBOTICS_CONTAINER_REGISTRY}" ]]; then
  echo "Usage: $0 <container-registry>"
  exit 1
fi

bazel run //src/app_charts:push "${CLOUD_ROBOTICS_CONTAINER_REGISTRY}"
bazel build //src/go/tests/apps:go_default_test

# Run the artifact directly so the kind clusters can be easily accessed for
# debugging.
cd ${DIR}/../../../../bazel-bin/src/go/tests/apps/go_default_test_/go_default_test.runfiles/cloud_robotics

ACCESS_TOKEN="$(gcloud auth application-default print-access-token)" \
  REGISTRY="${CLOUD_ROBOTICS_CONTAINER_REGISTRY}" \
  ../../go_default_test
