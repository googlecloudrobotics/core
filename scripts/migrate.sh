#!/bin/bash
#
# Copyright 2025 The Cloud Robotics Authors
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

# helper functions to update from older installations
# ./migrate.sh <migration> <project-id>

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "${DIR}/common.sh"
source "${DIR}/config.sh"
source "${DIR}/include-config.sh"

set -o pipefail -o errexit

# Required or terraform will fail deleting the IoT registry
function cleanup_iot_devices {
  gcloud services list --project="${GCP_PROJECT_ID}" | grep -q cloudiot.googleapis.com || return
  local iot_registry_name="cloud-robotics"
  gcloud beta iot registries list --project="${GCP_PROJECT_ID}" --region="${GCP_REGION}" | grep -q "${iot_registry_name}" || return
  local devices
  devices=$(gcloud beta iot devices list \
    --project "${GCP_PROJECT_ID}" \
    --region "${GCP_REGION}" \
    --registry "${iot_registry_name}" \
    --format='value(id)')
  if [[ -n "${devices}" ]] ; then
    echo "Clearing IoT devices from ${iot_registry_name}" 1>&2
    for dev in ${devices}; do
      gcloud beta iot devices delete \
        --quiet \
        --project "${GCP_PROJECT_ID}" \
        --region "${GCP_REGION}" \
        --registry "${iot_registry_name}" \
        ${dev}
    done
  fi
}

function cleanup_helm_data {
  # Delete all legacy HELM resources. Do not delete the Helm charts directly, as
  # we just want to keep the resources and have synk "adopt" them.
  kc delete cm ready-for-synk 2> /dev/null || true
  kc delete cm synk-enabled 2> /dev/null || true
  kc -n kube-system delete deploy tiller-deploy 2> /dev/null || true
  kc -n kube-system delete service tiller-deploy 2> /dev/null || true
  kc -n kube-system delete cm -l OWNER=TILLER 2> /dev/null || true
}

function cleanup_old_cert_manager {
  # Uninstall and cleanup older versions of cert-manager if needed

  echo "checking for old cert manager .."
  kc &>/dev/null get deployments cert-manager || return 0
  installed_ver=$(kc get deployments cert-manager -o=go-template --template='{{index .metadata.labels "helm.sh/chart"}}' | rev | cut -d'-' -f1 | rev | tr -d "vV")
  echo "have cert manager $installed_ver"

  if [[ "$installed_ver" == 0.5.* ]]; then
    echo "need to cleanup old version"

    # see https://docs.cert-manager.io/en/latest/tasks/upgrading/upgrading-0.5-0.6.html#upgrading-from-older-versions-using-helm
    # and https://docs.cert-manager.io/en/latest/tasks/backup-restore-crds.html

    # cleanup
    synk_version=$(kc get resourcesets.apps.cloudrobotics.com --output=name | grep cert-manager | cut -d'/' -f2)
    echo "deleting resourceset ${synk_version}"
    ${SYNK} delete ${synk_version} -n default
    kc delete crd \
      certificates.certmanager.k8s.io \
      issuers.certmanager.k8s.io \
      clusterissuers.certmanager.k8s.io
  fi

  if [[ "$installed_ver" == 0.8.* ]]; then
    echo "need to cleanup old version"
    # see https://cert-manager.io/docs/installation/upgrading/upgrading-0.8-0.9/
    # and https://cert-manager.io/docs/installation/upgrading/upgrading-0.9-0.10/

    # cleanup
    kc delete deployments --namespace default \
      cert-manager \
      cert-manager-cainjector \
      cert-manager-webhook

    kc delete -n default issuer cert-manager-webhook-ca cert-manager-webhook-selfsign
    kc delete -n default certificate cert-manager-webhook-ca cert-manager-webhook-webhook-tls
    kc delete apiservice v1beta1.admission.certmanager.k8s.io
  fi

  if [[ "$installed_ver" == 0.10.* ]]; then
    echo "need to cleanup old version"

    # cleanup deployments
    kc delete deployments --namespace default \
      cert-manager \
      cert-manager-cainjector \
      cert-manager-webhook

    echo "Wait until cert-manager pods are deleted"
    kc wait pods -l app.kubernetes.io/instance=cert-manager -n default --for=delete --timeout=35s

    # delete existing cert-manager resources
    kc delete Issuers,ClusterIssuers,Certificates,CertificateRequests,Orders,Challenges --all-namespaces --all

    # Delete old webhook ca and tls secrets
    kc delete secrets --namespace default cert-manager-webhook-ca cert-manager-webhook-tls

    # cleanup crds
    kc delete crd \
      certificaterequests.certmanager.k8s.io \
      certificates.certmanager.k8s.io \
      challenges.certmanager.k8s.io \
      clusterissuers.certmanager.k8s.io \
      issuers.certmanager.k8s.io \
      orders.certmanager.k8s.io

    # cleanup apiservices
    kc delete apiservices v1beta1.webhook.certmanager.k8s.io
  fi

  # This is now installed as part of base-cloud
  kc delete resourcesets.apps.cloudrobotics.com -l name=cert-manager 2>/dev/null || true
}

# main
if [[ "$#" -lt 2 ]] || [[ ! "$1" =~ ^(cleanup_helm_data|cleanup_old_cert_manager|cleanup_iot_devices)$ ]]; then
  die "Usage: $0 {cleanup_helm_data|cleanup_old_cert_manager|cleanup_iot_devices} <project id>"
fi

include_config $2

# log and call arguments verbatim:
log $2 $0 $1
"$@"
