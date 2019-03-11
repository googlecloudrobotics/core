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

# A utility for dumping service resources details
# ./scripts/dump-services.sh (<type> (<project-id>))
#
# - type: {target-pools|forwarding-rules|addresses|firewall-rules}
#         default is 'target-pools'
# - project-id: uses default gcloud project if not given
#
# ./scripts/dump-services.sh | sort -t# -k3
# ./scripts/dump-services.sh forwarding-rules | sort -t# -k3
#
# Stale services can be removed with
# ./delete-service <id> (<project-id>)

TYPE=${1:-"target-pools"}
PROJECT=${2:-$(gcloud 2>/dev/null config get-value project)}
REGION=$(gcloud 2>/dev/null config  get-value compute/region)
REGION=${REGION:-europe-west1}

LIST=$(gcloud --project="${PROJECT}" compute ${TYPE} list --format='value(name)')

for x in ${LIST}; do
  description=$(gcloud compute --project="${PROJECT}" ${TYPE} describe "${x}" --region="${REGION}" --format="json")
  service=$(echo "$description" | jq -r .description | jq -r '."kubernetes.io/service-name"')
  timestamp=$(echo "$description" | jq -r .creationTimestamp)
  echo "${x} # ${timestamp} # ${service}"
done
