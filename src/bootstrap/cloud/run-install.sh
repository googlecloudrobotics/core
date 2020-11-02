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

{ # this ensures the entire script is downloaded #

set -o pipefail -o errexit

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BUCKET_URI=${BUCKET_URI:-"https://storage.googleapis.com/cloud-robotics-releases"}
GCP_PROJECT_ID="$1"

if [[ -z "$2" || "$2" = --* ]]; then
  TARGET="latest"
  COMMAND="$2"
else
  TARGET="$2"
  COMMAND="$3"
fi

if [[ -z "${GCP_PROJECT_ID}" || ! "${COMMAND}" =~ ^(|--set-config|--set-oauth|--delete)$ ]]; then
  echo "Usage: $0 <project id> [<version-file>|<tarball>] [<command>]"
  echo "Supported commands:"
  echo "  --set-config    Updates the cloud config interactively."
  echo "  --set-oauth     Enables and configures OAuth interactively."
  echo "  --delete        Deletes Cloud Robotics from the cloud project."
  exit 1
fi

if [[ ! "${TARGET}" = *.tar.gz ]]; then
  echo "Downloading version file ${BUCKET_URI}/${TARGET}"
  TARGET=$( curl --silent --show-error --fail "${BUCKET_URI}/${TARGET}" )
fi

echo "Downloading tarball ${BUCKET_URI}/${TARGET}"
TMPDIR="$( mktemp -d )"
curl --silent --show-error --fail "${BUCKET_URI}/${TARGET}" | tar xz -C "${TMPDIR}"
cd ${TMPDIR}/cloud-robotics-core

# Pass xtrace on to subshells if set for this shell.
BASH=bash
if [[ $SHELLOPTS =~ xtrace ]] ; then
  BASH="bash -o xtrace"
fi

if [[ "${COMMAND}" = "--set-config" ]]; then
  $BASH scripts/set-config.sh "${GCP_PROJECT_ID}"
elif [[ "${COMMAND}" = "--set-oauth" ]]; then
  $BASH scripts/set-config.sh "${GCP_PROJECT_ID}" --edit-oauth
elif [[ "${COMMAND}" = "--delete" ]]; then
  $BASH ./deploy.sh delete "${GCP_PROJECT_ID}"
else
  # We tag the setup-robot files with this information to be able to check if
  # cloud and robot-installations are in sync
  export TARGET
  $BASH scripts/set-config.sh "${GCP_PROJECT_ID}" --ensure-config
  $BASH ./deploy.sh create "${GCP_PROJECT_ID}"
fi

cd ${DIR}
rm -rf ${TMPDIR}

} # this ensures the entire script is downloaded #
