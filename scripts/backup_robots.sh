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

if ! hash jq 2>/dev/null; then
  echo >&2 "This script needs jq (apt install jq)."
  exit 1
fi
if ! hash yq 2>/dev/null; then
  echo >&2 "This script needs yq (pip3 install yq)."
  exit 1
fi

KUBE_CONTEXT=${KUBE_CONTEXT:-minikube}

function kc {
  kubectl --context="${KUBE_CONTEXT}" "$@"
}

kc get robots -o yaml | \
  yq 2>/dev/null -ry '.items[] | del(.metadata.annotations["kubectl.kubernetes.io/last-applied-configuration"],.metadata.creationTimestamp,.metadata.generation,.metadata.managedFields,.metadata.resourceVersion,.metadata.selfLink,.metadata.uid,.status)' -

