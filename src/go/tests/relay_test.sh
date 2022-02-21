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
#
# This test only works in conjunction with a sim vm. E.g. from the top of the
# repo run:
# ./scripts/robot-sim.sh create "<myproject>" "sim1"
# bazel test --test_env GCP_PROJECT_ID="<myproject>" --test_env CLUSTER="sim1" --test_env HOME="${HOME}" --test_output=streamed --test_tag_filters="external" //src/go/tests:relay_test
#
# Add -v7 to kc exec in the tests to get more details when debugging.

CLUSTER="${CLUSTER:-test-robot}"
TEST_POD_NAME="busybox-sleep"
KC_CFG_DIR=$(mktemp -d -t kc-XXXXXXXXXX)
export KUBECONFIG="${KC_CFG_DIR}/test"
touch ${KUBECONFIG}

function kc() {
  kubectl --context="${CLUSTER}" "$@"
}

function setup() {
  kubectl config set-credentials "${GCP_PROJECT_ID}" --auth-provider gcp
  # setup relay for robot-sim vm (test-robot)
  kubectl config set-cluster "${CLUSTER}" --server="https://www.endpoints.${GCP_PROJECT_ID}.cloud.goog/apis/core.kubernetes-relay/client/${CLUSTER}"
  kubectl config set-context "${CLUSTER}" --cluster "${CLUSTER}" --namespace "default" --user "$GCP_PROJECT_ID"

  # delete test pod (if running)
  if kc get pod "${TEST_POD_NAME}" -o name 2>/dev/null; then
    kc delete pod --ignore-not-found "${TEST_POD_NAME}"
    kc wait --for=delete pod/"${TEST_POD_NAME}" --timeout=60s
  fi
  # deploy a container with a shell that runs sleep
  kc run "${TEST_POD_NAME}" --image=gcr.io/google-containers/busybox:latest --restart=Never -- /bin/sh -c "trap : TERM INT; sleep 3600 & wait"
  kc wait --for=condition=Ready pod/"${TEST_POD_NAME}"
}

function teardown() {
  # delete test pod (if running)
  kc delete pod --ignore-not-found "${TEST_POD_NAME}" || /bin/true
 
  rm -rf "${KC_CFG_DIR}"
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
  res=$(kc -v7 exec "${TEST_POD_NAME}" -- id)
  if [[ "$res" != "uid=0(root) gid=0(root) groups=10(wheel)" ]]; then
    test_failed "id command did not run, output was \"$res\""
  fi

  test_passed "id command worked"
}

function test_relay_handles_eof() {
  # pipe commands from stdin through the relay
  res=$({ echo "echo foo"; } | kc exec "${TEST_POD_NAME}" -i -- sh)
  if [[ "$res" != "foo" ]]; then
    test_failed "echo command did not run, output was \"$res\""
  fi

  test_passed "echo command worked"
}

setup
test_relay_can_exec_to_shell
test_relay_handles_eof

