#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "${DIR}/common.sh"

PROJECT_DIR="${DIR}/deployments/robco-navtest"
source "${PROJECT_DIR}/config.sh"

gcloud auth configure-docker --quiet

# TODO(skopecki) These variables should be declared in the run-install.sh and removed from this script.
export BUCKET_URI="https://storage.googleapis.com/robco-ci-binary-builds"
export SOURCE_CONTAINER_REGISTRY="gcr.io/robco-team"

# Deploy the binary release that was pushed by the last successful integration test.
curl --silent --show-error --fail-with-body "${BUCKET_URI}/run-install.sh" \
    | bash -x -s -- ${GCP_PROJECT_ID}

