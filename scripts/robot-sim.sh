#!/bin/bash
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

# Manage simulated robots
#
# Simulated robots are does as a separate cluster, running the same components like a physical robot
# in addition to the robot simulator.

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "${DIR}/common.sh"
source "${DIR}/include-config.sh"

set -o pipefail -o errexit

INITIAL_KUBECTL_CONTEXT="$(kubectl config current-context)"
GKE_CLOUD_CONTEXT="gke_${GCP_PROJECT_ID}_${GCP_ZONE}_cloud-robotics"
PROJECT_DOMAIN=${CLOUD_ROBOTICS_DOMAIN:-"www.endpoints.${GCP_PROJECT_ID}.cloud.goog"}
APP_MANAGEMENT=${APP_MANAGEMENT:-true}

if [[ -z "${ROBOT_LABELS}" ]]; then
  ROBOT_LABELS="simluated=true"
fi

function restore_initial_context {
  kubectl config use-context "${INITIAL_KUBECTL_CONTEXT}"
}

function create {
  local ROBOT_NAME="$1"
  local ROBOT_ROLE="${2:-Navtest (simulated)}"
  local ROBOT_TYPE="${3:-mir-100}"
  local GKE_SIM_CONTEXT="gke_${GCP_PROJECT_ID}_${GCP_ZONE}_${ROBOT_NAME}"

  # Create cloud cluster for robot simulation and restore original kubectl context
  gcloud >/dev/null 2>&1 container clusters describe "${ROBOT_NAME}" \
    --zone=${GCP_ZONE} --project=${GCP_PROJECT_ID} || \
  gcloud container clusters create "${ROBOT_NAME}" \
    --enable-legacy-authorization \
    --machine-type="n1-standard-8" \
    --num-nodes=1 \
    --max-nodes=2 \
    --scopes gke-default,https://www.googleapis.com/auth/cloud-platform \
    --zone=${GCP_ZONE} \
    --project=${GCP_PROJECT_ID}

  gcloud container clusters get-credentials "${ROBOT_NAME}" \
    --zone=${GCP_ZONE} --project=${GCP_PROJECT_ID}

  APP_MANAGEMENT="" \
  KUBE_CONTEXT=${GKE_SIM_CONTEXT} \
  ACCESS_TOKEN=$(gcloud auth application-default print-access-token) \
    $DIR/../src/bootstrap/robot/setup_robot.sh \
    ${ROBOT_NAME} \
    --domain ${PROJECT_DOMAIN} --project ${GCP_PROJECT_ID} \
    --robot-role "${ROBOT_ROLE}" \
    --robot-type "${ROBOT_TYPE}" --robot-authentication=false \
    --labels "${ROBOT_LABELS}" \
    --app-management="${APP_MANAGEMENT}"
}

function delete {
  local ROBOT_NAME="$1"

  kubectl --context=${GKE_CLOUD_CONTEXT} delete robots.registry.cloudrobotics.com "${ROBOT_NAME}" || true
  gcloud container clusters delete "${ROBOT_NAME}" \
    --zone=${GCP_ZONE} --project=${GCP_PROJECT_ID}
}

# Alias for create.
function update {
  create $@
}

# main

if [ "$#" -lt 2 ]; then
  die "Usage: $0 {create|delete|update} <robot-name> [<robot-role>]"
fi

# TODO(b/116303345): usage of 'gcloud' above silently switched the default context.
# Restore it after we exit.
trap restore_initial_context EXIT

# call arguments verbatim:
$@
