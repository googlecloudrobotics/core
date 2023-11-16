#!/bin/bash
#
# Presubmit script for testing cloud robotics.
# Expected to run remotely on a GitHub Actions runner, not locally.

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# shellcheck source=ci/common.sh
source "${DIR}/common.sh"

echo "Timestamp: build started"
bazel_ci build --nobuild  //...
echo "Timestamp: build-deps fetched"
bazel_ci build //...
echo "Timestamp: build done"
bazel_ci test --test_output=errors //...
echo "Timestamp: test done"

# Some of the tests below pull Docker images from the repository. We need to
# make sure they are pushed and provide an access token.
gcloud auth activate-service-account --key-file robco_integration_test_credentials.json
gcloud auth configure-docker --quiet

REGISTRY="gcr.io/robco-integration-test"
TAG="latest" bazel_ci run \
  //src/app_charts:push "${REGISTRY}"

set +o xtrace  # Don't put the access token in the logs.
ACCESS_TOKEN="$(GOOGLE_APPLICATION_CREDENTIALS=robco_integration_test_credentials.json gcloud auth application-default print-access-token)"
# --strategy=TestRunner=standalone means that the tests are run locally
# and not on a remote worker (which does not have the Docker environment).
bazel_ci test \
  --flaky_test_attempts 3 \
  --test_env ACCESS_TOKEN="${ACCESS_TOKEN}" \
  --test_env REGISTRY="${REGISTRY}" \
  --test_tag_filters="requires-docker" \
  --test_output=errors \
  --strategy=TestRunner=standalone //src/go/tests/apps:go_default_test

set -o xtrace

echo "Timestamp: presubmit.sh done"
