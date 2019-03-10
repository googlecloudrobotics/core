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

# A utility for deleting stale service resources
# See e.g. https://github.com/kubernetes/kubernetes/issues/39420

PROJECT=${1:-$(gcloud 2>/dev/null config get-value project)}
REGION=$(gcloud 2>/dev/null config  get-value compute/region)
REGION=${REGION:-europe-west1}

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 1.) stale services from previous deployments
# - this keep the most recent resource alive, since we don't know if this
#   still in use
function delete_stale_services {
  LIST=$(gcloud --project="${PROJECT}" compute target-pools list --format='value(name)')

  records=""
  # Build a list of "service;timestamp;id" to be able to detect old versions
  # TODO(ensonic): sometimes it seems the service is gone from the target-pools,
  # but there is still an entry in forwarding-rules
  for x in ${LIST}; do
    description=$(gcloud compute --project="${PROJECT}" target-pools describe "${x}" --region="${REGION}" --format="json")
    service=$(echo "$description" | jq -r .description | jq -r '."kubernetes.io/service-name"')
    timestamp=$(echo "$description" | jq -r .creationTimestamp)
    records=$(echo "$records"; echo "$service;$timestamp;$x")
  done

  # Check remaining records. Look for services with multiple ressource entries and
  # delete all but the most recent.
  for service in $(echo "$records" | cut -d';' -f1 | sort -u); do
    count=$(echo "$records" | grep -c "$service")
    if [[ "$count" -gt "1" ]]; then
      echo "== $service has $count copies =="
      while IFS= read -r old; do
        x=$(echo "$old" | cut -d';' -f3)
        ${DIR}/delete-service.sh "$x" ${PROJECT}
      done < <(echo "$records" | grep "$service" | sort | head -n-1)
    fi
  done
}

# 2.) stale robots-services that where deleted though the registry-api
# - we can only run this is the deployment is deleted or we know the list of
#   robots
function delete_stale_robots {
  # Whitelist of known robot services to keep
  # TODO(ensonic): consider getting this from the registry:
  # curl 'https://${CLOUD_ROBOTICS_DOMAIN}/apis/core.registry/v1alpha1/robots' | jq -r '.["robots"][] | .["id"]'
  # or set to an empty list to delete everything when the deployment is down
  # ROBOTS="5631943370604544"

  LIST=$(gcloud --project="${PROJECT}" compute target-pools list --format='value(name)')

  for x in ${LIST}; do
    description=$(gcloud compute --project="${PROJECT}" target-pools describe "${x}" --region="${REGION}" --format="json")
    service=$(echo "$description" | jq -r .description | jq -r '."kubernetes.io/service-name"')
    if echo "$service" | grep -q "default/robot"; then
      robot=$(echo "$service" | rev | cut -d'-' -f1 | rev)
      if echo "$ROBOTS" | grep -qv "$robot"; then
        ${DIR}/delete-service.sh "$x" ${PROJECT}
      fi
    fi
  done
}

if [ "$#" -lt 1 ]; then
  delete_stale_services
else
  # call arguments verbatim:
  # TODO(rodrigoq): this fails if you pass the project in $1. Should we do more
  # sophisticated argument parsing?
  "$@"
fi
