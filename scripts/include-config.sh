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

# Includes the configuration variables from a config.sh.

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

  if is_source_install; then
    # Keep default in sync with src/go/pkg/configutil/config-reader.go
    CLOUD_ROBOTICS_CONTAINER_REGISTRY=${CLOUD_ROBOTICS_CONTAINER_REGISTRY:-"gcr.io/${GCP_PROJECT_ID}"}
    SOURCE_CONTAINER_REGISTRY=${CLOUD_ROBOTICS_CONTAINER_REGISTRY}
  else
    SOURCE_CONTAINER_REGISTRY=${SOURCE_CONTAINER_REGISTRY:-gcr.io/cloud-robotics-releases}
  fi

  CLOUD_ROBOTICS_DEPLOY_ENVIRONMENT=${CLOUD_ROBOTICS_DEPLOY_ENVIRONMENT:-GCP}
  check_var_is_one_of CLOUD_ROBOTICS_DEPLOY_ENVIRONMENT "GCP" "GCP-testing"
}
