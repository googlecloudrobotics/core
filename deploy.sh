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

# Manage a deployment

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "${DIR}/scripts/common.sh"
source "${DIR}/scripts/include-config.sh"

set -o pipefail -o errexit

PROJECT_NAME="cloud-robotics"

if is_source_install; then
  TERRAFORM="${DIR}/bazel-out/../../../external/hashicorp_terraform/terraform"
  HELM_COMMAND="${DIR}/bazel-out/../../../external/kubernetes_helm/helm"
  # To avoid a dependency on the host's glibc, we build synk with pure="on".
  # Because this is a non-default build configuration, it results in a separate
  # subdirectory of bazel-out/, which is not as easy to hardcode as
  # bazel-bin/... Instead, we use `bazel run` to locate and execute the binary.
  SYNK_COMMAND="bazel ${BAZEL_FLAGS} run //src/go/cmd/synk --"
  TV_COMMAND="bazel ${BAZEL_FLAGS} run //src/go/cmd/token-vendor --"
else
  TERRAFORM="${DIR}/bin/terraform"
  HELM_COMMAND="${DIR}/bin/helm"
  SYNK_COMMAND="${DIR}/bin/synk"
  TV_COMMAND="${DIR}/bin/token-vendor"
fi

TERRAFORM_DIR="${DIR}/src/bootstrap/cloud/terraform"
TERRAFORM_APPLY_FLAGS=${TERRAFORM_APPLY_FLAGS:- -auto-approve}
# utility functions

function include_config_and_defaults {
  include_config "$1"

  CLOUD_ROBOTICS_DOMAIN=${CLOUD_ROBOTICS_DOMAIN:-"www.endpoints.${GCP_PROJECT_ID}.cloud.goog"}
  APP_MANAGEMENT=${APP_MANAGEMENT:-false}

  # lets-encrypt is used as the default certificate provider for backwards compatibility purposes
  CLOUD_ROBOTICS_CERTIFICATE_PROVIDER=${CLOUD_ROBOTICS_CERTIFICATE_PROVIDER:-lets-encrypt}
  CLOUD_ROBOTICS_CERTIFICATE_SUBJECT_COMMON_NAME=${CLOUD_ROBOTICS_CERTIFICATE_SUBJECT_COMMON_NAME:-GCP_PROJECT_ID}
  CLOUD_ROBOTICS_CERTIFICATE_SUBJECT_ORGANIZATION=${CLOUD_ROBOTICS_CERTIFICATE_SUBJECT_ORGANIZATION:-GCP_PROJECT_ID}

  CLOUD_ROBOTICS_OWNER_EMAIL=${CLOUD_ROBOTICS_OWNER_EMAIL:-$(gcloud config get-value account)}
  KUBE_CONTEXT="gke_${GCP_PROJECT_ID}_${GCP_ZONE}_${PROJECT_NAME}"

  HELM="${HELM_COMMAND} --kube-context ${KUBE_CONTEXT}"
  SYNK="${SYNK_COMMAND} --context ${KUBE_CONTEXT}"
}

function kc {
  kubectl --context="${KUBE_CONTEXT}" "$@"
}

function prepare_source_install {
  bazel ${BAZEL_FLAGS} build \
      "@hashicorp_terraform//:terraform" \
      "@kubernetes_helm//:helm" \
      //src/app_charts/base:base-cloud \
      //src/app_charts/platform-apps:platform-apps-cloud \
      //src/app_charts:push \
      //src/go/cmd/setup-robot:setup-robot.push \
      //src/go/cmd/synk

  # TODO(rodrigoq): the containerregistry API would be enabled by Terraform, but
  # that doesn't run until later, as it needs the digest of the setup-robot
  # image. Consider splitting prepare_source_install into source_install_build
  # and source_install_push and using Terraform to enable the API in between.
  gcloud services enable containerregistry.googleapis.com \
    --project "${GCP_PROJECT_ID}"

  # `setup-robot.push` is the first container push to avoid a GCR bug with parallel pushes on newly
  # created projects (see b/123625511).
  ${DIR}/bazel-bin/src/go/cmd/setup-robot/setup-robot.push \
    --dst="${CLOUD_ROBOTICS_CONTAINER_REGISTRY}/setup-robot:latest"

  # The tag variable must be called 'TAG', see cloud-robotics/bazel/container_push.bzl
  # Running :push outside the build system shaves ~3 seconds off an incremental build.
  TAG="latest" ${DIR}/bazel-bin/src/app_charts/push "${CLOUD_ROBOTICS_CONTAINER_REGISTRY}"
}

function terraform_exec {
  ( cd "${TERRAFORM_DIR}" && ${TERRAFORM} "$@" )
}

function terraform_cleanup {
  # Terraform doesn't seem to remove node pool taints without recreating the
  # node pool, whereas this approach causes less downtime.
  pool=(base-pool "--cluster=cloud-robotics" "--zone=${GCP_ZONE}" "--project=${GCP_PROJECT_ID}")
  if [[ -n "$(gcloud container node-pools describe "${pool[@]}" --format 'value(config.taints)')" ]] ; then
    gcloud beta container node-pools update "${pool[@]}" --quiet \
      --no-enable-autoscaling
    gcloud beta container node-pools update "${pool[@]}" --quiet \
      --node-taints=""
    gcloud beta container node-pools update "${pool[@]}" --quiet \
      --enable-autoscaling --min-nodes=2 --max-nodes=10
  fi
}

function terraform_init {
  if [[ -z "${PRIVATE_DOCKER_PROJECTS:-}" ]]; then
    # Transition helper: Until all configs have PRIVATE_DOCKER_PROJECTS,
    # default to CLOUD_ROBOTICS_CONTAINER_REGISTRY.
    if [[ -n "${CLOUD_ROBOTICS_CONTAINER_REGISTRY:-}" ]] && [[ "${CLOUD_ROBOTICS_CONTAINER_REGISTRY:-}" != "${GCP_PROJECT_ID}" ]]; then
      PRIVATE_DOCKER_PROJECTS="$(echo ${CLOUD_ROBOTICS_CONTAINER_REGISTRY} | sed -n -e 's:^.*gcr.io/::p')"
    fi
  fi

  local ROBOT_IMAGE_DIGEST
  ROBOT_IMAGE_DIGEST=$(cat bazel-bin/src/go/cmd/setup-robot/setup-robot.push.digest)

  # We only need to create dns resources if a custom domain is used.
  local CUSTOM_DOMAIN
  if [[ "${CLOUD_ROBOTICS_DOMAIN}" != "www.endpoints.${GCP_PROJECT_ID}.cloud.goog" ]]; then
    CUSTOM_DOMAIN="${CLOUD_ROBOTICS_DOMAIN}"
  fi

  # This variable is set by src/bootstrap/cloud/run-install.sh for binary installs
  local CRC_VERSION
  if [[ -z "${TARGET}" ]]; then
    # TODO(ensonic): keep this in sync with the nightly release script
    VERSION=${VERSION:-"0.1.0"}
    if [[ -d .git ]]; then
      SHA=$(git rev-parse --short HEAD)
    else
      echo "WARNING: no git dir and no \$TARGET env set"
      SHA="unknown"
    fi
    CRC_VERSION="crc-${VERSION}/crc-${VERSION}+${SHA}"
  else
    CRC_VERSION="${TARGET%.tar.gz}"
  fi

  cat > "${TERRAFORM_DIR}/terraform.tfvars" <<EOF
# autogenerated by deploy.sh, do not edit!
name = "${GCP_PROJECT_ID}"
id = "${GCP_PROJECT_ID}"
domain = "${CUSTOM_DOMAIN}"
zone = "${GCP_ZONE}"
region = "${GCP_REGION}"
shared_owner_group = "${CLOUD_ROBOTICS_SHARED_OWNER_GROUP}"
robot_image_reference = "${SOURCE_CONTAINER_REGISTRY}/setup-robot@${ROBOT_IMAGE_DIGEST}"
crc_version = "${CRC_VERSION}"
cr_syncer_rbac = "${CR_SYNCER_RBAC}"
certificate_provider = "${CLOUD_ROBOTICS_CERTIFICATE_PROVIDER}"
EOF

# Add certificate information if the configured provider requires it
  if [[ ! "none self-signed" =~ (" "|^)"${CLOUD_ROBOTICS_CERTIFICATE_PROVIDER}"(" "|$) ]]; then
    cat >> "${TERRAFORM_DIR}/terraform.tfvars" <<EOF
certificate_subject_common_name = "${CLOUD_ROBOTICS_CERTIFICATE_SUBJECT_COMMON_NAME}"
certificate_subject_organization = "${CLOUD_ROBOTICS_CERTIFICATE_SUBJECT_ORGANIZATION}"
EOF

  if [[ -n "${CLOUD_ROBOTICS_CERTIFICATE_SUBJECT_ORGANIZATIONAL_UNIT}" ]]; then
      cat >> "${TERRAFORM_DIR}/terraform.tfvars" <<EOF
certificate_subject_organizational_unit = "${CLOUD_ROBOTICS_CERTIFICATE_SUBJECT_ORGANIZATIONAL_UNIT}"
EOF
    fi
  fi

# Docker private projects

  if [[ -n "${PRIVATE_DOCKER_PROJECTS:-}" ]]; then
    cat >> "${TERRAFORM_DIR}/terraform.tfvars" <<EOF
private_image_repositories = ["${PRIVATE_DOCKER_PROJECTS// /\", \"}"]
EOF
  fi

# Terraform bucket

  if [[ -n "${TERRAFORM_GCS_BUCKET:-}" ]]; then
    cat > "${TERRAFORM_DIR}/backend.tf" <<EOF
# autogenerated by deploy.sh, do not edit!
terraform {
  backend "gcs" {
    bucket = "${TERRAFORM_GCS_BUCKET}"
    prefix = "${TERRAFORM_GCS_PREFIX}"
  }
}
EOF
  else
    rm -f "${TERRAFORM_DIR}/backend.tf"
  fi

  terraform_exec init -upgrade -reconfigure \
    || die "terraform init failed"
}

function terraform_apply {
  terraform_cleanup
  terraform_init

  # We've stopped managing Google Cloud projects in Terraform, make sure they
  # aren't deleted.
  terraform_exec state rm google_project.project 2>/dev/null || true
#  echo "Importing CA resources"
#  echo -n "CA: "
  terraform_exec state rm google_privateca_certificate_authority.ca
#  terraform_exec import google_privateca_certificate_authority.ca robco-ensonic/europe-west1/robco-ensonic-ca-pool/robco-ensonic-ca
#  echo -n "CA Pool: "
#  terraform_exec state rm google_privateca_ca_pool.ca_pool
#  terraform_exec import google_privateca_ca_pool.ca_pool robco-ensonic/europe-west1/robco-ensonic-ca-pool

  terraform_exec apply ${TERRAFORM_APPLY_FLAGS} \
    || die "terraform apply failed"
}

function terraform_delete {
  terraform_init
  terraform_exec destroy -auto-approve || die "terraform destroy failed"
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

function helm_cleanup {
  cleanup_helm_data
  cleanup_old_cert_manager
}

function helm_charts {
  local INGRESS_IP
  INGRESS_IP=$(terraform_exec output ingress-ip)

  local CURRENT_CONTEXT
  CURRENT_CONTEXT=$(kubectl config current-context 2>/dev/null) \
    || CURRENT_CONTEXT=""
  gcloud container clusters get-credentials "${PROJECT_NAME}" \
    --zone ${GCP_ZONE} \
    --project ${GCP_PROJECT_ID} \
    || die "create: failed to get cluster credentials"
  [[ -n "${CURRENT_CONTEXT}" ]] && kubectl config use-context "${CURRENT_CONTEXT}"


  # Wait for the GKE cluster to be reachable.
  i=0
  until kc get serviceaccount default &>/dev/null; do
    sleep 1
    i=$((i + 1))
    if ((i >= 60)) ; then
      # Try again, without suppressing stderr this time.
      if ! kc get serviceaccount default >/dev/null; then
        die "'kubectl get serviceaccount default' failed"
      fi
    fi
  done

  local BASE_NAMESPACE
  BASE_NAMESPACE="default"

  # Remove old unmanaged cert
  if ! kc get secrets cluster-authority -o yaml | grep -q "cert-manager.io/certificate-name: selfsigned-ca"; then
    kc delete secrets cluster-authority 2> /dev/null || true
  fi

  # Delete permissive binding if it exists from previous deployments
  if kc get clusterrolebinding permissive-binding &>/dev/null; then
    kc delete clusterrolebinding permissive-binding
  fi

  ${SYNK} init
  echo "synk init done"

    values=$(cat <<EOF
    --set-string domain=${CLOUD_ROBOTICS_DOMAIN}
    --set-string ingress_ip=${INGRESS_IP}
    --set-string project=${GCP_PROJECT_ID}
    --set-string region=${GCP_REGION}
    --set-string registry=${SOURCE_CONTAINER_REGISTRY}
    --set-string owner_email=${CLOUD_ROBOTICS_OWNER_EMAIL}
    --set-string app_management=${APP_MANAGEMENT}
    --set-string certificate_provider=${CLOUD_ROBOTICS_CERTIFICATE_PROVIDER}
    --set-string deploy_environment=${CLOUD_ROBOTICS_DEPLOY_ENVIRONMENT}
    --set-string oauth2_proxy.client_id=${CLOUD_ROBOTICS_OAUTH2_CLIENT_ID}
    --set-string oauth2_proxy.client_secret=${CLOUD_ROBOTICS_OAUTH2_CLIENT_SECRET}
    --set-string oauth2_proxy.cookie_secret=${CLOUD_ROBOTICS_COOKIE_SECRET}
    --set use_tv_verbose=${CRC_USE_TV_VERBOSE}
EOF
)

  echo "installing base-cloud to ${KUBE_CONTEXT}..."
  ${HELM} template -n base-cloud --namespace=${BASE_NAMESPACE} ${values} \
      ./bazel-bin/src/app_charts/base/base-cloud-0.0.1.tgz \
    | ${SYNK} apply base-cloud -n ${BASE_NAMESPACE} -f - \
    || die "Synk failed for base-cloud"

  echo "installing platform-apps-cloud to ${KUBE_CONTEXT}..."
  ${HELM} template -n platform-apps-cloud ${values} \
      ./bazel-bin/src/app_charts/platform-apps/platform-apps-cloud-0.0.1.tgz \
    | ${SYNK} apply platform-apps-cloud -f - \
    || die "Synk failed for platform-apps-cloud"
}

# commands

function set_config {
  local project_id="$1"
  ${DIR}/scripts/set-config.sh "${project_id}"
}

function create {
  include_config_and_defaults $1
  if is_source_install; then
    prepare_source_install
  fi
  terraform_apply
  helm_cleanup
  helm_charts
}

function delete {
  include_config_and_defaults $1
  if is_source_install; then
    bazel ${BAZEL_FLAGS} build "@hashicorp_terraform//:terraform"
  fi
  terraform_delete
}

# Alias for create.
function update {
  create $1
}

# This is a shortcut for skipping Terraform config checks if you know the config has not changed.
function fast_push {
  include_config_and_defaults $1
  if is_source_install; then
    prepare_source_install
  fi
  helm_charts
}

# This is a shortcut for skipping building and applying Terraform configs if you know the build has not changed.
function update_infra {
  include_config_and_defaults $1
  terraform_apply
}

# main
if [[ "$#" -lt 2 ]] || [[ ! "$1" =~ ^(set_config|create|delete|update|fast_push|update_infra)$ ]]; then
  die "Usage: $0 {set_config|create|delete|update|fast_push|update_infra} <project id>"
fi

# call arguments verbatim:
"$@"
