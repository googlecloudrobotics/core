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
# repo run: (one time)
#
#     ./scripts/robot-sim.sh create "${PROJECT:?}" "sim1"
#
# Then to deploy and test the relay:
#
#   ./deploy.sh fast_push "${PROJECT:?}"
#   sleep 30  # allow time for http-relay-server/client to update
#   bazel test --test_env GCP_PROJECT_ID="${PROJECT:?}" --test_env CLUSTER="sim1" --test_env HOME="${HOME}" --test_output=streamed --test_tag_filters="external" //src/go/tests:relay_test
#
# Instead of `sleep 30` you can watch:
#
#   kubectl --context gke_${PROJECT:?}_${ZONE:?}_cloud-robotics -n app-k8s-relay get pods -w
#   kubectl --context gke_${PROJECT:?}_${ZONE:?}_sim1 -n app-k8s-relay get pods -w
#
# Add -v7 to kc exec in the tests to get more details when debugging.

CLUSTER="${CLUSTER:-test-robot}"
TEST_POD_NAME="busybox-sleep"
KC_DIR=$(mktemp -d -t kc-XXXXXXXXXX)
export KUBECONFIG="${KC_DIR}/test"
touch "${KUBECONFIG}"
export KUBECACHE="${KC_DIR}/cache"
mkdir -p "${KUBECACHE}"

# gcloud expects to be able to write to its config directly.
CLOUDSDK_CONFIG=$(mktemp -d -t gcloud-XXXXXXXXXX)
export CLOUDSDK_CONFIG
cp -a ~/.config/gcloud/* "${CLOUDSDK_CONFIG}"

function kc() {
  kubectl --cache-dir="${KUBECACHE}" --context="${CLUSTER}" "$@"
}

function setup() {
  # configure kubectl to use relay for robot-sim vm (test-robot)
  kubectl config set-credentials "${GCP_PROJECT_ID}" --exec-command=gke-gcloud-auth-plugin --exec-api-version=client.authentication.k8s.io/v1beta1
  sed -i "s/provideClusterInfo: false/provideClusterInfo: true/" "${KUBECONFIG}"
  kubectl config set-cluster "${CLUSTER}" --server="https://www.endpoints.${GCP_PROJECT_ID}.cloud.goog/apis/core.kubernetes-relay/client/${CLUSTER}"
  kubectl config set-context "${CLUSTER}" --cluster "${CLUSTER}" --namespace "default" --user "$GCP_PROJECT_ID"
  echo "Checking relay is working..."
  kubectl --context "${CLUSTER}" version || test_failed "during setup, failed to reach the robot-sim VM"

  # delete test pod (if running)
  if kc get pod "${TEST_POD_NAME}" -o name 2>/dev/null; then
    kc delete pod --ignore-not-found "${TEST_POD_NAME}"
    kc wait --for=delete pod/"${TEST_POD_NAME}" --timeout=60s
  fi
  # deploy a container with a shell that runs sleep
  kc run "${TEST_POD_NAME}" --image=gcr.io/google-containers/busybox:1.27.2 --restart=Never -- /bin/sh -c "trap : TERM INT; sleep 3600 & wait"
  kc wait --for=condition=Ready pod/"${TEST_POD_NAME}"
}

function teardown() {
  # delete test pod (if running)
  kc delete pod --ignore-not-found "${TEST_POD_NAME}" || /bin/true

  rm -rf "${KC_CFG_DIR}" "${CLOUDSDK_CONFIG}"
}

function test_failed() {
  echo "TEST FAILED: $1"
  exit 1
}

function test_passed() {
  echo "TEST PASSED: $1"
}

function test_relay_can_exec_to_shell() {
  # exec command in shell-container through the relay
  res=$(kc exec "${TEST_POD_NAME}" -- echo hello)
  if [[ "$res" != "hello" ]]; then
    test_failed "echo command did not run, output was \"$res\", want \"hello\""
  fi

  test_passed "echo command worked"
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
trap teardown EXIT
test_relay_can_exec_to_shell
test_relay_handles_eof

