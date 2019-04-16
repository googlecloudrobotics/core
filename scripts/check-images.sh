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

#
# Run with 'json' or 'text' as a first arg to select the format.
#
# If you don't have the API enabled run (and wait a day to get results):
# gcloud --project ${GCP_PROJECT_ID} services enable containeranalysis.googleapis.com
#
# Postprocessing examples:
# - grep "Critical" /tmp/cve-check.demo.txt | sed -e 's/  Critical (\([0-9]*\)):/\1/g' | paste -s -d+ | bc
#
# Almost all of our images are built with distroless (bazel xxx_image rules). If there are
# vulnerabilities,
# 1.) check that we are using an up-to-date rules_docker in WORKSPACE. Check upstream
#     for recent commits that update the distroless base images and if that does not help
# 2.) check https://github.com/GoogleContainerTools/distroless/commits/master
#     for fixes, if there are some, clone rules_docker, run ./update_deps.sh and sent a PR.

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "${DIR}/common.sh"
source "${DIR}/include-config.sh"

function json {
  need_delimiter=0
  echo "["
  for image in $(gcloud container images list --format='csv[no-heading](name)' --repository=${CLOUD_ROBOTICS_CONTAINER_REGISTRY}); do
    if [[ $need_delimiter == 0 ]]; then
      need_delimiter=1
    else
      echo ","
    fi
    gcloud --project ${GCP_PROJECT_ID} alpha container images describe --show-package-vulnerability --format=json ${image}:${DOCKER_TAG} || \
      need_delimiter=0
  done
  echo "]"
}

function text {
  for image in $(gcloud container images list --format='csv[no-heading](name)' --repository=${CLOUD_ROBOTICS_CONTAINER_REGISTRY}); do
    # Filter noise
    gcloud --project ${GCP_PROJECT_ID}  alpha container images describe --show-package-vulnerability ${image}:${DOCKER_TAG} | \
        egrep -v '^\s*(registry|repository|digest):'
  done
}

if [[ -z "$2" ]]; then
  die "Usage: $0 <project-id>"
fi

include_config "$2"

# call arguments verbatim:
"$@"

