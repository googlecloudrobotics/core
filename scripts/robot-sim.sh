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

function set_defaults {
  local GCP_PROJECT_ID="$1"
  include_config "${GCP_PROJECT_ID}"
  INITIAL_KUBECTL_CONTEXT="$(kubectl config current-context)"
  GKE_CLOUD_CONTEXT="gke_${GCP_PROJECT_ID}_${GCP_ZONE}_cloud-robotics"

  if [[ -z "${ROBOT_LABELS}" ]]; then
    ROBOT_LABELS="simulated=true"
  fi
}

function restore_initial_context {
  kubectl config use-context "${INITIAL_KUBECTL_CONTEXT}"
}

function create {
  local GCP_PROJECT_ID="$1"
  local ROBOT_NAME="$2"
  local ROBOT_TYPE="${3:-mir-100}"

  set_defaults "${GCP_PROJECT_ID}"

  local GKE_SIM_CONTEXT="gke_${GCP_PROJECT_ID}_${GCP_ZONE}_${ROBOT_NAME}"

  # Create cloud cluster for robot simulation unless already exists.
  # To more accurately simulate a robot cluster, this uses the
  # robot-service@ service account instead of enabling Workload Identity, as we
  # don't have any on-prem/robot equivalent to that.
  gcloud >/dev/null 2>&1 container clusters describe "${ROBOT_NAME}" \
    --zone=${GCP_ZONE} --project=${GCP_PROJECT_ID} || \
  gcloud container clusters create "${ROBOT_NAME}" \
    --enable-legacy-authorization \
    --machine-type="e2-standard-2" \
    --num-nodes=1 \
    --max-nodes=2 \
    --enable-ip-alias \
    --issue-client-certificate \
    --no-enable-basic-auth \
    --metadata disable-legacy-endpoints=true \
    --scopes gke-default,cloud-platform \
    --service-account "robot-service@${GCP_PROJECT_ID}.iam.gserviceaccount.com" \
    --zone=${GCP_ZONE} \
    --project=${GCP_PROJECT_ID}

  gcloud container clusters get-credentials "${ROBOT_NAME}" \
    --zone=${GCP_ZONE} --project=${GCP_PROJECT_ID}

  # shellcheck disable=2097 disable=2098
  KUBE_CONTEXT=${GKE_SIM_CONTEXT} \
  HOST_HOSTNAME="nic0.${ROBOT_NAME}${GCP_ZONE}.c.${GCP_PROJECT_ID}.internal.gcpnode.com" \
  ACCESS_TOKEN=$(gcloud auth application-default print-access-token) \
    $DIR/../src/bootstrap/robot/setup_robot.sh \
    ${ROBOT_NAME} \
    --project ${GCP_PROJECT_ID} \
    --robot-type "${ROBOT_TYPE}" --robot-authentication=false \
    --labels "${ROBOT_LABELS}"
}

function delete {
  local GCP_PROJECT_ID="$1"
  local ROBOT_NAME="$2"

  set_defaults "${GCP_PROJECT_ID}"

  kubectl --context=${GKE_CLOUD_CONTEXT} delete robots.registry.cloudrobotics.com "${ROBOT_NAME}" || true
  gcloud container clusters delete "${ROBOT_NAME}" \
    --zone=${GCP_ZONE} --project=${GCP_PROJECT_ID}
}

# Alias for create.
function update {
  create "$@"
}

# main

if [[ "$#" -lt 3 ]]; then
  die "Usage: $0 {create|delete|update} <project-id> <robot-name> [<robot-type>]"
fi

# TODO(b/116303345): usage of 'gcloud' above silently switched the default context.
# Restore it after we exit.
trap restore_initial_context EXIT

# call arguments verbatim:
"$@"
