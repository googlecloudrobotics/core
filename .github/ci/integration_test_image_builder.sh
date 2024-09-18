#!/bin/bash
# Builds and pushes a docker image that can be used in Cloud Build to run
# the integration test (see integration_test_cloudbuild.yaml).
#
# To be manually invoked and the resulting sha256 copied to
# integration_test_cloudbuild.yaml after changing the Dockerfile.

set -euo pipefail

NAME="gcr.io/robco-integration-test/integration-test-image"
docker build --network=host -t "${NAME}" - \
  < .github/ci/Dockerfile.integration-test-image
docker push "${NAME}"
