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
#     * Allow setting up OAuth via an --oauth flag.
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

# Parse flags.
if [[ ! "$1" = --* ]]; then
  GCP_PROJECT_ID="$1"
fi

for arg in "$@"; do
  if [[ "${arg}" = "--local" ]]; then
    FLAG_LOCAL=1
  elif [[ "${arg}" = "--create-new" ]]; then
    FLAG_CREATE_NEW=1
  fi
done

if [[ -z "${FLAG_LOCAL}" && -z "${GCP_PROJECT_ID}" ]]; then
  echo
  echo "Usage: $0 <project id> [<options>]"
  echo "Supported options:"
  echo "  --local         Creates a local config. Doesn't require the project id."
  echo "  --create-new    Does nothing if a config exists already."
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
  if [[ -n "${FLAG_CREATE_NEW}" ]]; then
    echo "Found Cloud Robotics config."
    exit 0
  fi
  source ${CONFIG}
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

# Ask for region and zone.
GCP_ZONE=${GCP_ZONE:-"europe-west1-c"}
read_variable NEW_GCP_ZONE "In which zone should Cloud Robotics be deployed?" "${GCP_ZONE}"

# Verify the zone exists.
gcloud compute zones list --project ${GCP_PROJECT_ID} | grep "^${NEW_GCP_ZONE} \+" >/dev/null \
  || die "ERROR: the zone does not exist in your project: ${NEW_GCP_ZONE}"

NEW_GCP_REGION=${NEW_GCP_ZONE%-?}


# Ask for Terraform bucket and location.
if [[ -z "${TERRAFORM_GCS_BUCKET}" ]]; then
  TF_LOCATION="${CLOUD_BUCKET}/terraform"
else
  TF_LOCATION="gs://${TERRAFORM_GCS_BUCKET}/${TERRAFORM_GCS_PREFIX}"
fi
read_variable NEW_TF_LOCATION "In which GCP storage folder should your Terraform state be stored?" \
  "${TF_LOCATION}"
TF_LOCATION_REGEX='^\(gs://\)\?\([-_a-zA-Z0-9]\+\)\(\/\(.*[^/]\)\)\?/\?$'
NEW_TERRAFORM_GCS_BUCKET=$( echo "${NEW_TF_LOCATION}" | sed "s#${TF_LOCATION_REGEX}#\2#;q" ) \
  || die "ERROR: Invalid GCP storage folder. Accepted format: gs://<bucket>/<folder>"
NEW_TERRAFORM_GCS_PREFIX=$( echo "${NEW_TF_LOCATION}" | sed "s#${TF_LOCATION_REGEX}#\4#;q" )

# Output configuration before saving.
echo
echo "   Your configuration"
echo "========================"
print_variable "GCP project ID" "${GCP_PROJECT_ID}"
print_variable "GCP region" "${NEW_GCP_REGION}"
print_variable "GCP zone" "${NEW_GCP_ZONE}"
print_variable "Terraform state bucket" "${NEW_TERRAFORM_GCS_BUCKET}"
print_variable "Terraform state directory" "${NEW_TERRAFORM_GCS_PREFIX}"
echo

echo -n "Would you like to save this configuration? [Yn] "
while true; do
  read -n 1 input
  echo
  if [[ -z "${input}" || "${input}" =~ y|Y ]]; then
    break
  elif [[ "${input}" =~ n|N ]]; then
    exit 0
  fi
  echo -n "Please answer with 'y' or 'n'. "
done

if [[ -n "${TERRAFORM_GCS_BUCKET}" &&\
      ( ! "${TERRAFORM_GCS_BUCKET}" = "${NEW_TERRAFORM_GCS_BUCKET}" ||\
        ! "${TERRAFORM_GCS_PREFIX}" = "${NEW_TERRAFORM_GCS_PREFIX}") ]]; then
  # Copy Terraform state to new location.
  echo "Copying Terraform state..."
  gsutil cp "gs://${TERRAFORM_GCS_BUCKET}/${TERRAFORM_GCS_PREFIX}/*.tfstate" \
    "gs://${NEW_TERRAFORM_GCS_BUCKET}/${NEW_TERRAFORM_GCS_PREFIX}"
fi

# Save all parameter values.
echo "Saving configuration..."
if [[ ! -r ${CONFIG} ]]; then
  cp ${DIR}/config.sh.tmpl ${CONFIG}
fi
save_variable GCP_PROJECT_ID "${GCP_PROJECT_ID}"
save_variable GCP_REGION "${NEW_GCP_REGION}"
save_variable GCP_ZONE "${NEW_GCP_ZONE}"
save_variable TERRAFORM_GCS_BUCKET "${NEW_TERRAFORM_GCS_BUCKET}"
save_variable TERRAFORM_GCS_PREFIX "${NEW_TERRAFORM_GCS_PREFIX}"

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

