#!/bin/bash
#
# Copyright 2019 The Google Cloud Robotics Authors
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

# Deletes all GCE resources created for a Kubernetes service.

SERVICE="$1"
if [[ -z "$SERVICE" ]]; then
  echo "Usage: $0 <service> (<project-id>)"
  exit 1;
fi

PROJECT=${2:-$(gcloud 2>/dev/null config get-value project)}
REGION=$(gcloud 2>/dev/null config  get-value compute/region)
REGION=${REGION:-europe-west1}

gcloud compute --project="${PROJECT}" firewall-rules delete "k8s-fw-${SERVICE}" -q
gcloud compute --project="${PROJECT}" forwarding-rules delete "${SERVICE}" --region="${REGION}" -q
gcloud compute --project="${PROJECT}" addresses delete "${SERVICE}" --region="${REGION}" -q
gcloud compute --project="${PROJECT}" target-pools delete "${SERVICE}" --region="${REGION}" -q

