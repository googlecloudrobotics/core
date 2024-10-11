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

HELM="${TEST_SRCDIR}/+non_module_deps+kubernetes_helm/helm"
if [[ ! -x "${HELM}" ]] ; then
  # If we hit this again, consider using the runfiles library:
  # https://github.com/bazelbuild/bazel/blob/master/tools/bash/runfiles/runfiles.bash#L55-L86
  echo >&2 "Failed to locate helm in ${TEST_SRCDIR}."
  exit 1
fi

CLOUD_BASE="${TEST_SRCDIR}/_main/src/app_charts/base/base-cloud-0.0.1.tgz"
ROBOT_BASE="${TEST_SRCDIR}/_main/src/app_charts/base/base-robot-0.0.1.tgz"

function test_failed() {
  echo "TEST FAILED: $1"
  exit 1
}

function test_passed() {
  echo "TEST PASSED: $1"
}

function expect_app_installed() {
  local command="$1"
  local application="$2"
  local template
  if ! template=$(${command}); then
    test_failed "\"${command}\" failed"
  fi
  if [[ "${template}" != *"app: ${application}"* ]]; then
    echo "TEMPLATE: ${template}"
    test_failed "expected \"${application}\" to be installed in template created by \"${command}\""
  fi
  test_passed "application \"${application}\" is included in template created by \"${command}\""
}

function expect_app_not_installed() {
  local command="$1"
  local application="$2"
  local template
  if ! template=$(${command}); then
    echo "TEMPLATE: ${template}"
    test_failed "\"${command}\" failed"
  fi
  if [[ "${template}" == *"app: ${application}"* ]]; then
    test_failed "did not expected \"${application}\" to be installed in template created by \"${command}\""
  fi
  test_passed "application \"${application}\" is not included in template created by \"${command}\""
}

expect_app_installed "${HELM} template ${CLOUD_BASE} --set-string app_management=true" "app-rollout-controller"
expect_app_installed "${HELM} template ${CLOUD_BASE} --set-string app_management=true" "chart-assignment-controller"
expect_app_not_installed "${HELM} template ${CLOUD_BASE} --set-string app_management=false" "app-rollout-controller"
expect_app_not_installed "${HELM} template ${CLOUD_BASE} --set-string app_management=false" "chart-assignment-controller"

expect_app_installed "${HELM} template ${ROBOT_BASE} --set-string app_management=true" "chart-assignment-controller"
expect_app_not_installed "${HELM} template ${ROBOT_BASE} --set-string app_management=false" "chart-assignment-controller"
