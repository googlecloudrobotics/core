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
source "${DIR}/scripts/config.sh"
source "${DIR}/scripts/include-config.sh"

set -o pipefail -o errexit

PROJECT_NAME="cloud-robotics"
RUNFILES_ROOT="_main"

if is_source_install; then
  # Not using bazel run to not clobber the bazel-bin dir
  TERRAFORM="${DIR}/bazel-out/../../../external/+non_module_deps+hashicorp_terraform/terraform"
  HELM_COMMAND="${DIR}/bazel-out/../../../external/+non_module_deps+kubernetes_helm/helm"
  SYNK_COMMAND="${DIR}/bazel-bin/src/go/cmd/synk/synk_/synk"
else
  TERRAFORM="${DIR}/bin/terraform"
  HELM_COMMAND="${DIR}/bin/helm"
  SYNK_COMMAND="${DIR}/bin/synk"
fi

TERRAFORM_DIR="${DIR}/src/bootstrap/cloud/terraform"
TERRAFORM_APPLY_FLAGS=${TERRAFORM_APPLY_FLAGS:- -auto-approve}
# utility functions

function include_config_and_defaults {
  include_config "$1"

  CLOUD_ROBOTICS_DOMAIN=${CLOUD_ROBOTICS_DOMAIN:-"www.endpoints.${GCP_PROJECT_ID}.cloud.goog"}
  APP_MANAGEMENT=${APP_MANAGEMENT:-false}
  ONPREM_FEDERATION=${ONPREM_FEDERATION:-true}
  GKE_SECRET_MANAGER_PLUGIN=${GKE_SECRET_MANAGER_PLUGIN:-false}

  # lets-encrypt is used as the default certificate provider for backwards compatibility purposes
  CLOUD_ROBOTICS_CERTIFICATE_PROVIDER=${CLOUD_ROBOTICS_CERTIFICATE_PROVIDER:-lets-encrypt}
  CLOUD_ROBOTICS_CERTIFICATE_SUBJECT_COMMON_NAME=${CLOUD_ROBOTICS_CERTIFICATE_SUBJECT_COMMON_NAME:-GCP_PROJECT_ID}
  CLOUD_ROBOTICS_CERTIFICATE_SUBJECT_ORGANIZATION=${CLOUD_ROBOTICS_CERTIFICATE_SUBJECT_ORGANIZATION:-GCP_PROJECT_ID}

  CLOUD_ROBOTICS_OWNER_EMAIL=${CLOUD_ROBOTICS_OWNER_EMAIL:-$(gcloud config get-value account)}
  CLOUD_ROBOTICS_CTX=${CLOUD_ROBOTICS_CTX:-"gke_${GCP_PROJECT_ID}_${GCP_ZONE}_${PROJECT_NAME}"}

  SYNK="${SYNK_COMMAND} --context ${CLOUD_ROBOTICS_CTX}"
}

function update_config_var {
  cloud_bucket="gs://${1}-cloud-robotics-config"
  name="${2}"
  value="${3}"

  config_file="$(mktemp)"
  gsutil cp "${cloud_bucket}/config.sh" "${config_file}" 2>/dev/null || return

  save_variable "${config_file}" "${name}" "${value}"

  gsutil mv "${config_file}" "${cloud_bucket}/config.sh"
}

function prepare_source_install {
  # For whatever reasons different combinations of bazel environemnt seem to
  # work differently wrt bazel-bin. This hack ensure that both synk and the
  # files that synk will install are in bazel-bin
  tmpdir="$(mktemp -d)"
  bazel ${BAZEL_FLAGS} build //src/go/cmd/synk
  cp -a ${DIR}/bazel-bin/src/go/cmd/synk/synk_/synk ${tmpdir}/synk

  bazel ${BAZEL_FLAGS} build \
      "@hashicorp_terraform//:terraform" \
      "@kubernetes_helm//:helm" \
      //src/app_charts/base:base-cloud \
      //src/app_charts/platform-apps:platform-apps-cloud \
      //src/app_charts:push \
      //src/bootstrap/cloud:setup-robot.digest \
      //src/go/cmd/setup-robot:setup-robot.push

  mkdir -p ${DIR}/bazel-bin/src/go/cmd/synk/synk_/
  mv -n ${tmpdir}/synk ${DIR}/bazel-bin/src/go/cmd/synk/synk_/synk
  rm -f ${tmpdir}/synk
  rmdir ${tmpdir} || /bin/true

  # TODO(rodrigoq): the artifactregistry API would be enabled by Terraform, but
  # that doesn't run until later, as it needs the digest of the setup-robot
  # image. Consider splitting prepare_source_install into source_install_build
  # and source_install_push and using Terraform to enable the API in between.
  gcloud services enable artifactregistry.googleapis.com \
    --project "${GCP_PROJECT_ID}"

  # `setup-robot.push` is the first container push to avoid a GCR bug with parallel pushes on newly
  # created projects (see b/123625511).
  local oldPwd
  oldPwd=$(pwd)
  cd ${DIR}/bazel-bin/src/go/cmd/setup-robot/push_setup-robot.push.sh.runfiles/${RUNFILES_ROOT}
  ${DIR}/bazel-bin/src/go/cmd/setup-robot/push_setup-robot.push.sh \
    --repository="${CLOUD_ROBOTICS_CONTAINER_REGISTRY}/setup-robot" \
    --tag="latest"

  # The tag variable must be called 'TAG', see cloud-robotics/bazel/container_push.bzl
  # Running :push outside the build system shaves ~3 seconds off an incremental build.
  cd ${DIR}/bazel-bin/src/app_charts/push.runfiles/${RUNFILES_ROOT}
  TAG="latest" ${DIR}/bazel-bin/src/app_charts/push "${CLOUD_ROBOTICS_CONTAINER_REGISTRY}"
  cd ${oldPwd}
}

function terraform_exec {
  ( cd "${TERRAFORM_DIR}" && ${TERRAFORM} "$@" )
}

function terraform_init {
  local ROBOT_IMAGE_DIGEST
  ROBOT_IMAGE_DIGEST=$(cat bazel-bin/src/bootstrap/cloud/setup-robot.digest)

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
certificate_provider = "${CLOUD_ROBOTICS_CERTIFICATE_PROVIDER}"
cluster_type = "${GKE_CLUSTER_TYPE}"
datapath_provider = "${GKE_DATAPATH_PROVIDER}"
onprem_federation = ${ONPREM_FEDERATION}
secret_manager_plugin = ${GKE_SECRET_MANAGER_PLUGIN}
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

  echo 'additional_regions = {' >> "${TERRAFORM_DIR}/terraform.tfvars"
  local AR
  for AR in "${ADDITIONAL_REGIONS[@]}"; do
    local AR_NAME
    local AR_REGION
    local AR_ZONE

    AR_NAME=$(jq -r .name <<<"${AR}")
    AR_REGION=$(jq -r .region <<<"${AR}")
    AR_ZONE=$(jq -r .zone <<<"${AR}")

    cat >> "${TERRAFORM_DIR}/terraform.tfvars" <<EOF
    ${AR_NAME} = { region: "${AR_REGION}", zone: "${AR_ZONE}" },
EOF
  done
  echo '}' >> "${TERRAFORM_DIR}/terraform.tfvars"

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
  terraform_init

  terraform_exec apply ${TERRAFORM_APPLY_FLAGS} \
    || die "terraform apply failed"
  terraform_post
}

function terraform_post {
  local OLD_CLOUD_ROBOTICS_CTX
  local location

  OLD_CLOUD_ROBOTICS_CTX="${CLOUD_ROBOTICS_CTX}"
  CLOUD_ROBOTICS_CTX=$(gke_context_name "${GCP_PROJECT_ID}" "cloud-robotics" "${GCP_REGION}" "${GCP_ZONE}")
  [[ -z "${CLOUD_ROBOTICS_CTX}" ]] && die "no cloud-robotics cluster found"
  if [[ "${OLD_CLOUD_ROBOTICS_CTX}" != "${CLOUD_ROBOTICS_CTX}" ]]; then
    echo "updating CLOUD_ROBOTICS_CTX from ${OLD_CLOUD_ROBOTICS_CTX} to ${CLOUD_ROBOTICS_CTX}"
    update_config_var ${GCP_PROJECT_ID} "CLOUD_ROBOTICS_CTX" "${CLOUD_ROBOTICS_CTX}"
  fi
}

function terraform_delete {
  terraform_init
  terraform_exec destroy -auto-approve || die "terraform destroy failed"
}

function helm_region_shared {
  local CLUSTER_CONTEXT
  local CLUSTER_DOMAIN
  local INGRESS_IP
  local CLUSTER_REGION
  local CLUSTER_ZONE
  local CLUSTER_NAME

  CLUSTER_CONTEXT="${1}"
  CLUSTER_DOMAIN="${2}"
  INGRESS_IP="${3}"
  CLUSTER_REGION="${4}"
  CLUSTER_ZONE="${5}"
  CLUSTER_NAME="${6}"

  gke_get_credentials "${GCP_PROJECT_ID}" "${CLUSTER_NAME}" "${CLUSTER_REGION}" "${CLUSTER_ZONE}"

  # Wait for the GKE cluster to be reachable.
  i=0
  until kubectl --context "${CLUSTER_CONTEXT}" get serviceaccount default &>/dev/null; do
    sleep 1
    i=$((i + 1))
    if ((i >= 60)) ; then
      # Try again, without suppressing stderr this time.
      if ! kubectl --context "${CLUSTER_CONTEXT}" get serviceaccount default >/dev/null; then
        die "'kubectl get serviceaccount default' failed"
      fi
    fi
  done

  local BASE_NAMESPACE
  BASE_NAMESPACE="default"

  # Remove old unmanaged cert
  if ! kubectl --context "${CLUSTER_CONTEXT}" get secrets cluster-authority -o yaml | grep -q "cert-manager.io/certificate-name: selfsigned-ca"; then
    kubectl --context "${CLUSTER_CONTEXT}" delete secrets cluster-authority 2> /dev/null || true
  fi

  # Delete permissive binding if it exists from previous deployments
  if kubectl --context "${CLUSTER_CONTEXT}" get clusterrolebinding permissive-binding &>/dev/null; then
    kubectl --context "${CLUSTER_CONTEXT}" delete clusterrolebinding permissive-binding
  fi


  local values
  values=(
    --set-string "domain=${CLUSTER_DOMAIN}"
    --set-string "ingress_ip=${INGRESS_IP}"
    --set-string "project=${GCP_PROJECT_ID}"
    --set-string "region=${CLUSTER_REGION}"
    --set-string "registry=${SOURCE_CONTAINER_REGISTRY}"
    --set-string "owner_email=${CLOUD_ROBOTICS_OWNER_EMAIL}"
    --set-string "app_management=${APP_MANAGEMENT}"
    --set-string "onprem_federation=${ONPREM_FEDERATION}"
    --set-string "certificate_provider=${CLOUD_ROBOTICS_CERTIFICATE_PROVIDER}"
    --set-string "deploy_environment=${CLOUD_ROBOTICS_DEPLOY_ENVIRONMENT}"
    --set-string "oauth2_proxy.client_id=${CLOUD_ROBOTICS_OAUTH2_CLIENT_ID}"
    --set-string "oauth2_proxy.client_secret=${CLOUD_ROBOTICS_OAUTH2_CLIENT_SECRET}"
    --set-string "oauth2_proxy.cookie_secret=${CLOUD_ROBOTICS_COOKIE_SECRET}"
    --set "use_tv_verbose=${CRC_USE_TV_VERBOSE}"
  )

  ${SYNK_COMMAND} --context "${CLUSTER_CONTEXT}" init
  echo "synk init done"

  echo "installing base-cloud to ${CLUSTER_CONTEXT}..."
  ${HELM_COMMAND} --kube-context "${CLUSTER_CONTEXT}" template -n base-cloud --namespace=${BASE_NAMESPACE} "${values[@]}" \
      ./bazel-bin/src/app_charts/base/base-cloud-0.0.1.tgz \
    | ${SYNK_COMMAND} --context "${CLUSTER_CONTEXT}" apply base-cloud -n ${BASE_NAMESPACE} -f - \
    || die "Synk failed for base-cloud"

  # This is the main region. Only run this here!
  if [[ "${CLUSTER_NAME}" = "${PROJECT_NAME}" ]]; then
    echo "installing platform-apps-cloud to ${CLOUD_ROBOTICS_CTX}..."
    ${HELM_COMMAND} --kube-context "${CLUSTER_CONTEXT}" template -n platform-apps-cloud "${values[@]}" \
        ./bazel-bin/src/app_charts/platform-apps/platform-apps-cloud-0.0.1.tgz \
      | ${SYNK_COMMAND} --context "${CLUSTER_CONTEXT}" apply platform-apps-cloud -f - \
      || die "Synk failed for platform-apps-cloud"
  fi
}

function helm_main_region {
  local INGRESS_IP
  INGRESS_IP=$(terraform_exec output ingress-ip | tr -d '"')

  helm_region_shared \
    "${CLOUD_ROBOTICS_CTX}" \
    "${CLOUD_ROBOTICS_DOMAIN}" \
    "${INGRESS_IP}" \
    "${GCP_REGION}" \
    "${GCP_ZONE}" \
    "${PROJECT_NAME}"
}

function helm_additional_region {
  local ar_description
  ar_description="${1}"

  local AR_NAME
  local AR_REGION
  local AR_ZONE

  AR_NAME=$(jq -r .name <<<"${ar_description}")
  AR_REGION=$(jq -r .region <<<"${ar_description}")
  AR_ZONE=$(jq -r .zone <<<"${ar_description}")

  local CLUSTER_NAME
  CLUSTER_NAME="${AR_NAME}-ar-cloud-robotics"

  local INGRESS_IP
  INGRESS_IP=$(terraform_exec output -json ingress-ip-ar | jq -r ."\"${CLUSTER_NAME}\"")

  helm_region_shared \
    $(gke_context_name "${GCP_PROJECT_ID}" "${CLUSTER_NAME}" "${AR_REGION}" "${AR_ZONE}") \
    "${AR_NAME}.${CLOUD_ROBOTICS_DOMAIN}" \
    "${INGRESS_IP}" \
    "${AR_REGION}" \
    "${AR_ZONE}" \
    "${CLUSTER_NAME}"
}

function helm_charts {
  helm_main_region

  local AR
  for AR in "${ADDITIONAL_REGIONS[@]}"; do
    helm_additional_region "${AR}"
  done
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

# log and call arguments verbatim:
log $2 $0 $1
"$@"
