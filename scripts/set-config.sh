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

# TODO(skopecki)
#     * Consider setting CLOUD_ROBOTICS_SHARED_OWNER_GROUP and
#       APP_MANAGEMENT as well.

set -o pipefail -o errexit

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

source "${DIR}/scripts/common.sh"

# Escapes the input "foo bar" -> "foo\ bar".
function escape {
  sed 's/[^a-zA-Z0-9,._+@%/-]/\\&/g' <<< "$@"
}

# Escapes the input twice "foo bar" -> "foo\\\ bar"
function double_escape {
  sed 's/[^a-zA-Z0-9,._+@%/-]/\\\\\\&/g' <<< "$@"
}

# Reads a variable from user input.
function read_variable {
  local target_var="$1"
  local question="$2"
  local default="$3"

  echo
  echo "${question}"
  if [[ -n "${default}" ]]; then
    echo -n "[ENTER] for \"${default}\": "
  fi
  read -er input

  if [[ -z "${input}" ]]; then
    # shellcheck disable=SC2046
    eval ${target_var}=$( escape ${default} )
  else
    # shellcheck disable=SC2046
    eval ${target_var}=$( escape ${input} )
  fi
}

# Outputs the variable to the user.
function print_variable {
  local description="$1"
  local value="$2"

  if [[ -n "${value}" ]]; then
    echo "${description}: ${value}"
  fi
}

# Creates a substitution pattern for sed using an unprintable char as seperator.
# This allows the user to use any normal char in the input.
function sed_pattern {
  local regexp="$1"
  local replacement="$2"
  echo s$'\001'${regexp}$'\001'${replacement}$'\001'
}

# Sets the given variable in config.sh. If $value is empty, the variable
# assignement is commented out in config.sh.
function save_variable {
  local config_file="$1"
  local name="$2"
  local value="$3"

  if [[ -z "${value}" ]]; then
    sed -i "s/^\(${name}=.*\)$/#\1/" "${config_file}"
  elif grep -q "^\(# *\)\{0,1\}${name}=" "${config_file}"; then
    value=$( double_escape ${value} )
    sed -i "$( sed_pattern "^\(# *\)\{0,1\}${name}=.*$" "${name}=${value}" )" "${config_file}"
  else
    value=$( escape ${value} )
    echo >>"${config_file}"
    echo "${name}=${value}" >>"${config_file}"
  fi
}

# Asks a yes/no question and returns the mapped input.
function ask_yn {
  local question="$1"
  local default="$2"

  echo
  echo -n "$question"
  if [[ "${default}" = "n" ]]; then
    echo -n " [yN] "
  else
    echo -n " [Yn] "
  fi

  while true; do
    read -n 1 input
    if [[ -z "${input}" ]]; then
      return "$( [[ ! "${default}" = "n" ]] )"
    fi
    echo
    if [[ "${input}" =~ y|Y ]]; then
      return 0
    elif [[ "${input}" =~ n|N ]]; then
      return 1
    fi
    echo -n "Please answer with 'y' or 'n'. "
  done
}

# Parse flags.
if [[ ! "$1" = --* ]]; then
  GCP_PROJECT_ID="$1"
fi

for arg in "$@"; do
  if [[ "${arg}" = "--ensure-config" ]]; then
    FLAG_ENSURE_CONFIG=1
  elif [[ "${arg}" = "--edit-oauth" ]]; then
    FLAG_EDIT_OAUTH=1
  fi
done

if [[ -z "${GCP_PROJECT_ID}" ]]; then
  echo
  echo "Usage: $0 <project id> [<options>]"
  echo "Supported options:"
  echo "  --ensure-config    Does nothing if a config exists already."
  echo "  --edit-oauth       Enables and configures OAuth."
  die
fi

# Load config if it exists.
CLOUD_BUCKET="gs://${GCP_PROJECT_ID}-cloud-robotics-config"

CONFIG_FILE="$(mktemp)"
trap '{ rm -f ${CONFIG_FILE}; }' EXIT

if gsutil cp "${CLOUD_BUCKET}/config.sh" "${CONFIG_FILE}" 2>/dev/null; then
  if [[ -n "${FLAG_ENSURE_CONFIG}" ]]; then
    echo "Found Cloud Robotics config."
    exit 0
  fi
  source ${CONFIG_FILE}
else
  if [[ -n "${FLAG_EDIT_OAUTH}" ]]; then
    die "You have to create a config before you can enable OAuth."
  fi
  cp ${DIR}/config.sh.tmpl ${CONFIG_FILE}
fi

CLOUD_BUCKET="gs://${GCP_PROJECT_ID}-cloud-robotics-config"

# Check that the project exists and we have access.
gcloud projects describe "${GCP_PROJECT_ID}" >/dev/null \
  || die "ERROR: unable to access Google Cloud project: ${GCP_PROJECT_ID}"

function set_default_vars {
  # Enable Compute Engine API which is necessary to validate the zones.
  if ! gcloud services list --enabled --project ${GCP_PROJECT_ID} \
        | grep "^compute.googleapis.com \+" >/dev/null; then
    # TODO(skopecki) This can take a minute. Find a better solution to verify compute zones.
    echo "Enabling Compute Engine API..."
    gcloud services enable compute.googleapis.com --project ${GCP_PROJECT_ID}
  fi

  # Ask for region and zone.
  GCP_ZONE=${GCP_ZONE:-"europe-west1-c"}
  read_variable GCP_ZONE "In which zone should Cloud Robotics be deployed?" "${GCP_ZONE}"

  # Verify the zone exists.
  gcloud compute zones list -q --project ${GCP_PROJECT_ID} | grep "^${GCP_ZONE} \+" >/dev/null \
    || die "ERROR: the zone does not exist in your project: ${GCP_ZONE}"

  GCP_REGION=${GCP_ZONE%-?}

  # Ask for Terraform bucket and location.
  OLD_TERRAFORM_GCS_BUCKET="${TERRAFORM_GCS_BUCKET}"
  OLD_TERRAFORM_GCS_PREFIX="${TERRAFORM_GCS_PREFIX}"
  if [[ -z "${TERRAFORM_GCS_BUCKET}" ]]; then
    OLD_TF_LOCATION="${CLOUD_BUCKET}/terraform"
  else
    OLD_TF_LOCATION="gs://${TERRAFORM_GCS_BUCKET}/${TERRAFORM_GCS_PREFIX}"
  fi
  read_variable TF_LOCATION "In which GCP storage folder should your Terraform state be stored?" \
    "${OLD_TF_LOCATION}"
  TF_LOCATION_REGEX='^\(gs://\)\?\([-_a-zA-Z0-9]\+\)\(\/\(.*[^/]\)\)\?/\?$'
  TERRAFORM_GCS_BUCKET=$( echo "${TF_LOCATION}" | sed "s#${TF_LOCATION_REGEX}#\2#;q" ) \
    || die "ERROR: Invalid GCP storage folder. Accepted format: gs://<bucket>/<folder>"
  TERRAFORM_GCS_PREFIX=$( echo "${TF_LOCATION}" | sed "s#${TF_LOCATION_REGEX}#\4#;q" )

  # Docker registries.
  if is_source_install; then
    read_variable CLOUD_ROBOTICS_CONTAINER_REGISTRY \
      "Which Docker registry do you want to use when installing from sources? Use \"default\" for gcr.io/${GCP_PROJECT_ID}." \
      "${CLOUD_ROBOTICS_CONTAINER_REGISTRY:-default}"
    if [[ "${CLOUD_ROBOTICS_CONTAINER_REGISTRY}" == "default" ]]; then
      CLOUD_ROBOTICS_CONTAINER_REGISTRY=
    fi
  fi

  # TODO(skopecki) If CLOUD_ROBOTICS_CONTAINER_REGISTRY is private and does not belongs to this GCR project,
  #     it could be added automatically to PRIVATE_DOCKER_PROJECTS.
  read_variable PRIVATE_DOCKER_PROJECTS \
    "Do you need to read private Docker images from a GCR project? Space-separated list of alphanumeric project ids, or \"none\" for none." \
    "${PRIVATE_DOCKER_PROJECTS:-none}"
  if [[ "${PRIVATE_DOCKER_PROJECTS}" == "none" ]]; then
    PRIVATE_DOCKER_PROJECTS=
  fi
}

function set_oauth_vars {
  echo "Follow https://googlecloudrobotics.github.io/core/how-to/setting-up-oauth.html to obtain OAuth client id and secret."

  read_variable CLOUD_ROBOTICS_OAUTH2_CLIENT_ID "Enter OAuth client id." \
    "${CLOUD_ROBOTICS_OAUTH2_CLIENT_ID}"

  read_variable CLOUD_ROBOTICS_OAUTH2_CLIENT_SECRET "Enter OAuth client secret." \
    "${CLOUD_ROBOTICS_OAUTH2_CLIENT_SECRET}"

  if [[ -z "${CLOUD_ROBOTICS_COOKIE_SECRET}" ]] ||\
      ask_yn "Generate new cookie secret?" "n"; then
    CLOUD_ROBOTICS_COOKIE_SECRET="$( head -c 16 /dev/urandom | base64 )"
  fi
}

if [[ -n "${FLAG_EDIT_OAUTH}" ]]; then
  set_oauth_vars
else
  set_default_vars
fi

# Output configuration before saving.
echo
echo "   Your configuration"
echo "========================"
print_variable "GCP project ID" "${GCP_PROJECT_ID}"
print_variable "GCP region" "${GCP_REGION}"
print_variable "GCP zone" "${GCP_ZONE}"
print_variable "Terraform state bucket" "${TERRAFORM_GCS_BUCKET}"
print_variable "Terraform state directory" "${TERRAFORM_GCS_PREFIX}"
print_variable "Docker container registry" "${CLOUD_ROBOTICS_CONTAINER_REGISTRY}"
print_variable "Projects for private Docker images" "${PRIVATE_DOCKER_PROJECTS}"
print_variable "OAuth client id" "${CLOUD_ROBOTICS_OAUTH2_CLIENT_ID}"
print_variable "OAuth client secret" "${CLOUD_ROBOTICS_OAUTH2_CLIENT_SECRET}"
print_variable "OAuth cookie secret" "${CLOUD_ROBOTICS_COOKIE_SECRET}"

if ! ask_yn "Would you like to save this configuration?"; then
  exit 0
fi

if [[ -n "${OLD_TERRAFORM_GCS_BUCKET}" &&\
      ( ! "${OLD_TERRAFORM_GCS_BUCKET}" = "${TERRAFORM_GCS_BUCKET}" ||\
        ! "${OLD_TERRAFORM_GCS_PREFIX}" = "${TERRAFORM_GCS_PREFIX}") ]]; then
  # Copy Terraform state to new location.
  echo "Copying Terraform state..."
  gsutil cp "gs://${OLD_TERRAFORM_GCS_BUCKET}/${OLD_TERRAFORM_GCS_PREFIX}/*.tfstate" \
    "gs://${TERRAFORM_GCS_BUCKET}/${TERRAFORM_GCS_PREFIX}/"
fi

# Save all parameter values.
echo
echo "Saving configuration..."
save_variable "${CONFIG_FILE}" GCP_PROJECT_ID "${GCP_PROJECT_ID}"
save_variable "${CONFIG_FILE}" GCP_REGION "${GCP_REGION}"
save_variable "${CONFIG_FILE}" GCP_ZONE "${GCP_ZONE}"
save_variable "${CONFIG_FILE}" TERRAFORM_GCS_BUCKET "${TERRAFORM_GCS_BUCKET}"
save_variable "${CONFIG_FILE}" TERRAFORM_GCS_PREFIX "${TERRAFORM_GCS_PREFIX}"
save_variable "${CONFIG_FILE}" CLOUD_ROBOTICS_CONTAINER_REGISTRY "${CLOUD_ROBOTICS_CONTAINER_REGISTRY}"
save_variable "${CONFIG_FILE}" PRIVATE_DOCKER_PROJECTS "${PRIVATE_DOCKER_PROJECTS}"
save_variable "${CONFIG_FILE}" CLOUD_ROBOTICS_OAUTH2_CLIENT_ID "${CLOUD_ROBOTICS_OAUTH2_CLIENT_ID}"
save_variable "${CONFIG_FILE}" CLOUD_ROBOTICS_OAUTH2_CLIENT_SECRET "${CLOUD_ROBOTICS_OAUTH2_CLIENT_SECRET}"
save_variable "${CONFIG_FILE}" CLOUD_ROBOTICS_COOKIE_SECRET "${CLOUD_ROBOTICS_COOKIE_SECRET}"

# Upload config to the cloud.
if ! gsutil ls -p ${GCP_PROJECT_ID} | grep "^${CLOUD_BUCKET}/$" >/dev/null; then
  gsutil mb -p ${GCP_PROJECT_ID} ${CLOUD_BUCKET}
fi
gsutil mv "${CONFIG_FILE}" "${CLOUD_BUCKET}/config.sh"

if is_source_install && [[ ! -r "${DIR}/config.bzl" ]]; then
  # Create config.bzl.
  sed "s/my-project/${GCP_PROJECT_ID}/" "${DIR}/config.bzl.tmpl" > "${DIR}/config.bzl"
fi

