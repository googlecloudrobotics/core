#!/bin/bash
#
# Copyright 2021 The Cloud Robotics Authors
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

# see https://github.com/kubernetes/kubernetes/issues/90066#issuecomment-780236185 for hiding managed-fields
# another tool: https://github.com/itaysk/kubectl-neat

if ! hash jq; then
  echo "This script needs jq (apt install jq)."
  exit
fi
if ! hash yq; then
  echo "This script needs yq (pip3 install yq)."
  exit
fi

KUBE_CONTEXT=${KUBE_CONTEXT:-minikube}
LAST_APPLIED="kubectl.kubernetes.io/last-applied-configuration"

function kc {
  kubectl --context="${KUBE_CONTEXT}" "$@"
}

for r in $(kc get robots -o name); do
  echo "---"
  yaml=$(kc get $r -o yaml)
  if echo ${yaml} | grep -q "${LAST_APPLIED}"; then
    echo "${yaml}" | \
      yq 2>/dev/null -ry ".metadata.annotations[\"${LAST_APPLIED}\"]" -
   else
     echo "${yaml}" | \
       yq -ry 'del(.metadata.creationTimestamp,.metadata.generation,.metadata.managedFields,.metadata.resourceVersion,.metadata.selfLink,.metadata.uid,.status)' -
   fi
done

