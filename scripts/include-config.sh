#!/usr/bin/env bash
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

# Includes the configuration variables from a config.sh and ./config.bzl.

rootdir="$(dirname "${BASH_SOURCE[0]}")/.."
configbzl="${rootdir}/config.bzl"

function check_vars_not_empty {
  for v in "$@"; do
    [ -n "${!v}" ] || die "Variable $v is not set or is empty"
  done
}

function check_var_is_one_of {
  local var_name="$1"
  local allowed_values="${*:2}"
  local found=false

  for allowed_value in ${allowed_values}; do
    if [[ "${!var_name}" = "${allowed_value}" ]]; then
      found=true
    fi
  done

  if [[ "${found}" = false ]]; then
    die "Variable ${var_name} has to be one of [${allowed_values}], but was ${!var_name}"
  fi
}

function include_config {
  local project="$1"

  source <(gsutil cat "gs://${project}-cloud-robotics-config/config.sh")

  # Check that config defines the following set of configuration variables
  check_vars_not_empty GCP_PROJECT_ID GCP_REGION GCP_ZONE

  # Only import $configbzl for source installs
  if is_source_install; then
    if [[ ! -r $configbzl ]] ; then
      echo "ERROR: config.bzl does not exist or is not readable" >&2
      exit 1
    fi
    # Import config.bzl variables
    # shellcheck disable=2046
    # For better or worse, all spaces are removed by sed.
    export $(sed -e 's/[\ "]//g' -e '/^#/d' $configbzl)

    # Check that $configbzl defines the following union set of
    # configuration variables
    check_vars_not_empty DOCKER_TAG CLOUD_ROBOTICS_CONTAINER_REGISTRY
  fi

  CLOUD_ROBOTICS_DEPLOY_ENVIRONMENT=${CLOUD_ROBOTICS_DEPLOY_ENVIRONMENT:-GCP}
  check_var_is_one_of CLOUD_ROBOTICS_DEPLOY_ENVIRONMENT "GCP" "GCP-testing"

  # TODO(skopecki) SOURCE_CONTAINER_REGISTRY will be stored in config.sh
  #     in future and will replace CLOUD_ROBOTICS_CONTAINER_REGISTRY in
  #     the deploy script.
  if [[ -z "${SOURCE_CONTAINER_REGISTRY:-}" ]]; then
    SOURCE_CONTAINER_REGISTRY=${CLOUD_ROBOTICS_CONTAINER_REGISTRY:-gcr.io/cloud-robotics-releases}
  fi
}
