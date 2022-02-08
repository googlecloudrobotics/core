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

CLUSTER="test-robot"
TEST_POD_NAME="busybox-sleep"

function setup() {
  kubectl config set-credentials "${GCP_PROJECT_ID}" --auth-provider gcp
  # setup relay for robot-sim vm (test-robot)
  kubectl config set-cluster "${CLUSTER}" --server="https://www.endpoints.${GCP_PROJECT_ID}.cloud.goog/apis/core.kubernetes-relay/client/${CLUSTER}"
  kubectl config set-context "${CLUSTER}" --cluster "${CLUSTER}" --namespace "default" --user "$GCP_PROJECT_ID"

  # delete test pod (if running)
  if kubectl 2>/dev/null --context="${CLUSTER}" get pod "${TEST_POD_NAME}" -o name; then
    kubectl --context="${CLUSTER}" delete pod --ignore-not-found "${TEST_POD_NAME}"
    kubectl --context="${CLUSTER}" wait --for=delete pod/"${TEST_POD_NAME}" --timeout=60s
  fi
  # deploy a container with a shell that runs sleep
  kubectl --context="${CLUSTER}" run "${TEST_POD_NAME}" --image=gcr.io/google-containers/busybox:latest --restart=Never -- /bin/sh -c "trap : TERM INT; sleep 3600 & wait"
  kubectl --context="${CLUSTER}" wait --for=condition=Ready pod/"${TEST_POD_NAME}"
}

function teardown() {
  # delete test pod (if running)
  kubectl --context="${CLUSTER}" delete pod --ignore-not-found "${TEST_POD_NAME}" || /bin/true

  kubectl config delete-cluster "${CLUSTER}" || /bin/true
  kubectl config delete-context "${CLUSTER}" || /bin/true
}
trap teardown EXIT

function test_failed() {
  echo "TEST FAILED: $1"
  exit 1
}

function test_passed() {
  echo "TEST PASSED: $1"
}

function test_relay_can_exec_to_shell() {
  # exec command in shell-container through the relay
  res=$(kubectl --context="${CLUSTER}" exec "${TEST_POD_NAME}" -- id)
  if [[ "$res" != "uid=0(root) gid=0(root) groups=10(wheel)" ]]; then
    test_failed "id command did not run"
  fi

  test_passed "id command worked"
}

setup
test_relay_can_exec_to_shell

