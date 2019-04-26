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

# This script is a convenience wrapper for starting the setup-robot container, i.e., for doing
# "kubectl run ... --image=...setup-robot...".

set -e
set -o pipefail

if [[ -n "$APP_MANAGEMENT" ]] ; then
  echo "WARNING: using the APP_MANAGEMENT envvar for setup_robot.sh is deprecated." >&2
  echo "    Please use --app-management instead. Setup continues in 60 seconds..." >&2
  # Sleep, as the warning can't be seen after the helm output fills the screen.
  sleep 60
  APP_MANAGEMENT="--app-management=${APP_MANAGEMENT}"
else
  APP_MANAGEMENT=""
fi

if [[ ! "$*" =~ "--project" && $# -ge 2 ]] ; then
  echo "WARNING: using only positional arguments for setup_robot.sh is deprecated." >&2
  echo "    Please use the following invocation instead. Setup continues in 60 seconds..." >&2
  echo "    setup-robot <robot-name> --project <project-id> \\" >&2
  echo "        [--robot-type <type>] [--app-management]" >&2
  # Sleep, as the warning can't be seen after the helm output fills the screen.
  sleep 60
  # Rewrite parameters to new usage.
  set -- "$2" --project "$1" --robot-type "${3:-}"
fi

# Extract the project from the command-line args. It is required to identify the reference for the
# setup-robot image. This is challenging as --project is an option, so we have to do some
# rudimentary CLI parameter parsing.
for i in $(seq 1 $#) ; do
  if [[ "${!i}" == "--project" ]] ; then
    j=$((i+1))
    PROJECT=${!j}
  fi
done

if [[ -z "$PROJECT" ]] ; then
  echo "ERROR: --project <project-id> is required" >&2
  exit 1
fi

if [[ -z "${KUBE_CONTEXT}" ]] ; then
  KUBE_CONTEXT=kubernetes-admin@kubernetes
fi

# Full reference to the setup-robot image. The default is filled in by a genrule.
IMAGE_REFERENCE=$(curl -fsSL \
  "https://storage.googleapis.com/${PROJECT}-robot/setup_robot_image_reference.txt") || \
  IMAGE_REFERENCE=""

if [[ -z "$IMAGE_REFERENCE" ]] ; then
  echo "ERROR: failed to get setup_robot_image_reference.txt from GCS" >&2
  exit 1
fi

if [[ -n "$ACCESS_TOKEN_FILE" ]]; then
  ACCESS_TOKEN=$(cat ${ACCESS_TOKEN_FILE})
fi

if [[ -z "$ACCESS_TOKEN" ]]; then
  echo "Generate access token with gcloud:"
  echo "    gcloud auth application-default print-access-token"
  echo "Enter access token:"
  read ACCESS_TOKEN
fi

# Extract registry from IMAGE_REFERENCE. E.g.:
# IMAGE_REFERENCE = "eu.gcr.io/my-project/setup-robot@sha256:07...5465244d"
# REGISTRY = "eu.gcr.io"
REGISTRY=${IMAGE_REFERENCE%%/*}

# TODO(daschmidt): Remove the login dance when the setup-robot image is available from a public registry.
echo "Pulling image from ${REGISTRY}..."

echo ${ACCESS_TOKEN} | docker login -u oauth2accesstoken --password-stdin https://${REGISTRY} || true

if ! docker pull ${IMAGE_REFERENCE}; then
  docker logout https://${REGISTRY}
  exit 1
fi
docker logout https://${REGISTRY}

# Explicitly specify the context to not run this against the cloud cluster.
kubectl --context="${KUBE_CONTEXT}" run setup-robot --restart=Never -i --rm \
  --image=${IMAGE_REFERENCE} --env="ACCESS_TOKEN=${ACCESS_TOKEN}" -- \
   ${APP_MANAGEMENT} "$@"
