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

# Lists Kubernetes-related GCE usage. Run before and after an action to see if
# the action leaks.
# Regarding the quotas, see also:
# https://console.cloud.google.com/iam-admin/quotas
#
# If any of this is over the quotas, check cleanup-services.sh

PROJECT=${1:-$(gcloud 2>/dev/null config get-value project)}

color_def="\e[39m"
color_bad="\e[91m"
color_ok="\e[92m"

quota_full=0

quota_list="$(gcloud --project="${PROJECT}" compute project-info describe --format=json)"

function check {
  local type="$1"
  local readable_type
  readable_type="$(echo "${type}" | tr 'A-Z_' 'a-z ')"
  local quota
  quota="$(echo "${quota_list}" | jq ".[\"quotas\"] [] | select(.metric == \"${type}\")")"
  if [ -n "${quota}" ]; then
    local limit
    limit="$(echo "${quota}" | jq -r '.["limit"]')"
    local usage
    usage="$(echo "${quota}" | jq -r '.["usage"]')"
    local color=${color_ok}
    if [ ${usage} -eq ${limit} ]; then
      color=${color_bad}
      quota_full=1
    fi
    printf "%b%3d/%3d%b: ${readable_type}\n" ${color} ${usage} ${limit} ${color_def}
  else
    printf "  ?/  ?: ${readable_type}\n"
  fi
}

check FIREWALLS
check FORWARDING_RULES
check IN_USE_ADDRESSES
check TARGET_POOLS
check CPUS_ALL_REGIONS

exit ${quota_full}
