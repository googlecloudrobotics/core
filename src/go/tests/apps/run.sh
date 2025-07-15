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

# NOTE: if you are running a minikube using vmdriver=none on the same machine
# this test (or more precisely kind) does not seem to work - see
# https://github.com/kubernetes-sigs/kind/issues/2516

set -e

echo "Set NO_TEARDOWN=y for preserve \"kind\" clusters past test failures"
echo "run 'docker ps | grep kind' and 'docker stop <container-id>' to cleanup when done"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

CLOUD_ROBOTICS_CONTAINER_REGISTRY=${CLOUD_ROBOTICS_CONTAINER_REGISTRY:-$1}
if [[ -z "${CLOUD_ROBOTICS_CONTAINER_REGISTRY}" ]]; then
  echo "Usage: $0 <container-registry>"
  exit 1
fi

TAG="latest" bazel run //src/app_charts:push "${CLOUD_ROBOTICS_CONTAINER_REGISTRY}"
bazel build //src/go/tests/apps:go_default_test

# Run the artifact directly so the kind clusters can be easily accessed for
# debugging.
cd ${DIR}/../../../../bazel-bin/src/go/tests/apps/go_default_test_/go_default_test.runfiles/_main

ACCESS_TOKEN="$(gcloud auth application-default print-access-token)" \
  REGISTRY="${CLOUD_ROBOTICS_CONTAINER_REGISTRY}" \
  ../../go_default_test
