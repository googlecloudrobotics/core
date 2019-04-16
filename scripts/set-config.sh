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
  local name="$1"
  local value="$2"

  if [[ -z "${value}" ]]; then
    sed -i "s/^\(${name}=.*\)$/#\1/" "${CONFIG}"
  elif grep -q "^\(# *\)\{0,1\}${name}=" "${CONFIG}"; then
    value=$( double_escape ${value} )
    sed -i "$( sed_pattern "^\(# *\)\{0,1\}${name}=.*$" "${name}=${value}" )" "${CONFIG}"
  else
    value=$( escape ${value} )
    echo >>"${CONFIG}"
    echo "${name}=${value}" >>"${CONFIG}"
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
  if [[ "${arg}" = "--local" ]]; then
    FLAG_LOCAL=1
  elif [[ "${arg}" = "--ensure-config" ]]; then
    FLAG_ENSURE_CONFIG=1
  elif [[ "${arg}" = "--edit-oauth" ]]; then
    FLAG_EDIT_OAUTH=1
  fi
done

if [[ -z "${FLAG_LOCAL}" && -z "${GCP_PROJECT_ID}" ]]; then
  echo
  echo "Usage: $0 <project id> [<options>]"
  echo "Supported options:"
  echo "  --local            Creates a local config. Doesn't require the project id."
  echo "  --ensure-config    Does nothing if a config exists already."
  echo "  --edit-oauth       Enables and configures OAuth."
  die
fi

# Load config if it exists.
CLOUD_BUCKET="gs://${GCP_PROJECT_ID}-cloud-robotics-config"

if [[ -z "${FLAG_LOCAL}" ]]; then
  CONFIG="$( mktemp ).sh"
  # Download config if it exists. Otherwise, a new file is created later.
  gsutil cp "${CLOUD_BUCKET}/config.sh" "${CONFIG}" 2>/dev/null || true
else
  CONFIG=${CONFIG:-"${DIR}/config.sh"}
fi

if [[ -r ${CONFIG} ]]; then
  if [[ -n "${FLAG_ENSURE_CONFIG}" ]]; then
    echo "Found Cloud Robotics config."
    exit 0
  fi
  source ${CONFIG}
else
  if [[ -n "${FLAG_EDIT_OAUTH}" ]]; then
    die "You have to create a config before you can enable OAuth."
  fi
fi

# Ask for project ID, if was not not given as argument or in the local config.
if [[ -z "${GCP_PROJECT_ID}" ]]; then
  read_variable GCP_PROJECT_ID "What is the ID of your Google Cloud Platform project?" \
    "${GCP_PROJECT_ID}"
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

  read_variable PRIVATE_DOCKER_PROJECTS "Do you need to read private Docker images from a GCR project? Space-separated list of numeric project ids, or 'none' for none." \
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
if [[ ! -r ${CONFIG} ]]; then
  cp ${DIR}/config.sh.tmpl ${CONFIG}
fi
save_variable GCP_PROJECT_ID "${GCP_PROJECT_ID}"
save_variable GCP_REGION "${GCP_REGION}"
save_variable GCP_ZONE "${GCP_ZONE}"
save_variable TERRAFORM_GCS_BUCKET "${TERRAFORM_GCS_BUCKET}"
save_variable TERRAFORM_GCS_PREFIX "${TERRAFORM_GCS_PREFIX}"
save_variable CLOUD_ROBOTICS_OAUTH2_CLIENT_ID "${CLOUD_ROBOTICS_OAUTH2_CLIENT_ID}"
save_variable CLOUD_ROBOTICS_OAUTH2_CLIENT_SECRET "${CLOUD_ROBOTICS_OAUTH2_CLIENT_SECRET}"
save_variable CLOUD_ROBOTICS_COOKIE_SECRET "${CLOUD_ROBOTICS_COOKIE_SECRET}"
save_variable PRIVATE_DOCKER_PROJECTS "${PRIVATE_DOCKER_PROJECTS}"

if [[ -z "${FLAG_LOCAL}" ]]; then
  # Upload config to the cloud.
  if ! gsutil ls -p ${GCP_PROJECT_ID} | grep "^${CLOUD_BUCKET}/$" >/dev/null; then
    gsutil mb -p ${GCP_PROJECT_ID} ${CLOUD_BUCKET}
  fi
  gsutil mv "${CONFIG}" "${CLOUD_BUCKET}/config.sh"
fi

if is_source_install && [[ ! -r "${DIR}/config.bzl" ]]; then
  # Create config.bzl.
  sed "s/my-project/${GCP_PROJECT_ID}/" "${DIR}/config.bzl.tmpl" > "${DIR}/config.bzl"
fi

