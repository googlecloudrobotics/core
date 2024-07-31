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

function die {
  echo "$1" >&2
  exit 1
}

function is_source_install {
  # This file is present in the root folder only when installing from a binary.
  [[ ! -e "$(dirname "${BASH_SOURCE[0]}")/../INSTALL_FROM_BINARY" ]]
}

function log {
  local project
  project=$1
  shift
  gcloud logging write cloud-robotics-deploy \
      --severity=INFO \
      --project=${project} \
      --payload-type=json \
      "$(cat <<EOF
{
  "message": "$*",
  "user": "${USER}",
  "hostname": "$(hostname)",
  "git-branch": "$(git branch --show-current)",
  "git-ref": "$(git rev-parse --short HEAD)"
}
EOF
)"
}

# Fetch credentials for a new GKE cluster
function gke_get_credentials {
  local project
  project="$1"
  local cluster_name
  name="$2"
  local region
  region="$3"
  local zone
  zone="$4"

  # TODO(b/116303345): usage of 'gcloud ... get-credentials' silently switches
  # the default context.
  local saved_ctx
  saved_ctx=$(kubectl config current-context 2>/dev/null) || saved_ctx=""
  trap "[[ -n \"${saved_ctx}\" ]] && kubectl config use-context \"${saved_ctx}\"; trap - RETURN" RETURN

  local location
  location=$(gcloud container clusters list --filter="name=${name}" --format='value(location)' --project="${project}")
  case "${location}" in
    ${region})
      gcloud container clusters get-credentials "${name}" \
        --region "${region}" \
        --project "${project}" \
      ;;
    ${zone})
      gcloud container clusters get-credentials "${name}" \
        --zone "${zone}" \
        --project "${project}" \
      ;;
  esac
}