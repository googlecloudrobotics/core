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

set -o pipefail -o errexit
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd ${DIR}

function die {
  echo "$1" >&2
  exit 1
}

function push_image {
  local target=$1

  docker build -f "${target}/Dockerfile" -t "greeter-${target}" .
  docker tag "greeter-${target}" "gcr.io/${PROJECT_ID}/greeter-${target}"
  docker push "gcr.io/${PROJECT_ID}/greeter-${target}"
}

function create_config {
  cat greeter-server.yaml.tmpl | envsubst >greeter-server.yaml
}

# public functions
function push_client {
  push_image client
}

function update_config {
  create_config
  kubectl apply -f greeter-server.yaml
}

function update_server {
  push_image server
  kubectl delete pod -l 'app=greeter-server-app'
  update_config
}

function create {
  push_image server
  push_client
  update_config
}

function delete {
  create_config
  kubectl delete -f greeter-server.yaml
}

# main
if [[ -z ${PROJECT_ID} ]]; then
  die "Set PROJECT_ID first: export PROJECT_ID=[GCP project id]"
fi

if [[ ! "$1" =~ ^(create|delete|update_config|update_server|push_client)$ ]]; then
  die "Usage: $0 {create|delete|update_config|update_server|push_client}"
fi

# call arguments verbatim:
"$@"
