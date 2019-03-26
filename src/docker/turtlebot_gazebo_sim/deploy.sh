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

if [[ "$#" -lt 1 ]]; then
  echo "Usage: $0 <container-registry>/<project-id>"
  exit 1
fi

CLOUD_ROBOTICS_CONTAINER_REGISTRY="$1"
GIT_SHA=$(git rev-parse --short HEAD)
IMAGE_NAME="turtlebot3-gazebo-headless/${GIT_SHA}"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

docker build \
  -t "${IMAGE_NAME}" \
  -t "${CLOUD_ROBOTICS_CONTAINER_REGISTRY}/${IMAGE_NAME}" \
  ${DIR}
docker push "${CLOUD_ROBOTICS_CONTAINER_REGISTRY}/${IMAGE_NAME}"

IMAGE_DIGEST=$(docker image inspect "${IMAGE_NAME}" --format "{{.ID}}")
echo
echo "Image ${CLOUD_ROBOTICS_CONTAINER_REGISTRY}/${IMAGE_NAME} released as ${IMAGE_DIGEST}"
