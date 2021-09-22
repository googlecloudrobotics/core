#!/usr/bin/env bash

HELM="external/kubernetes_helm/helm template"
CLOUD_BASE="src/app_charts/base/base-cloud-0.0.1.tgz"
ROBOT_BASE="src/app_charts/base/base-robot-0.0.1.tgz"

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

expect_app_installed "${HELM} ${CLOUD_BASE} --set-string app_management=true" "app-rollout-controller"
expect_app_installed "${HELM} ${CLOUD_BASE} --set-string app_management=true" "chart-assignment-controller"
expect_app_not_installed "${HELM} ${CLOUD_BASE} --set-string app_management=false" "app-rollout-controller"
expect_app_not_installed "${HELM} ${CLOUD_BASE} --set-string app_management=false" "chart-assignment-controller"

expect_app_installed "${HELM} ${ROBOT_BASE} --set-string app_management=true" "chart-assignment-controller"
expect_app_not_installed "${HELM} ${ROBOT_BASE} --set-string app_management=false" "chart-assignment-controller"
