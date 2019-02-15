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

# Usage: ./install.sh [cloud-storage-bucket-url] [versioned-tarball]
if [[ $# -ge 2 ]]; then
  GCP_BUCKET=${1%/}
  TARBALL=$2
elif [[ $# -eq 1 ]]; then
  if [[ $1 =~ ^https?://.* ]]; then
    # User entered a url.
    GCP_BUCKET=${1%/}
  else
    TARBALL=$1
  fi
fi

GCP_BUCKET=${GCP_BUCKET:-"https://storage.googleapis.com/cloud-robotics-releases"}
# TODO(skopecki): Download the latest, stable version by default.
TARBALL=${TARBALL:-"cloud-robotics-core.tar.gz"}

echo "Downloading ${GCP_BUCKET}/${TARBALL}"
curl "${GCP_BUCKET}/${TARBALL}" | tar xvz

(cd ${DIR}/cloud-robotics-core && ./deploy.sh create)
