#!/bin/bash

set -euo pipefail

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# shellcheck source=ci/common.sh
source "${DIR}/common.sh"

gcloud auth activate-service-account --key-file cloud_robotics_releases_credentials.json
gcloud auth configure-docker --quiet

# Set defaults used for the release, they can only be overriden when testing
# manually.
GCP_BUCKET=${GCP_BUCKET:-"cloud-robotics-releases"}
VERSION=${VERSION:-"0.1.0"}
SHA=$(git rev-parse --short "$FULL_SHA")
RELEASE_NAME="v$VERSION-$SHA"
# TAG is a global variable that is used in the container push rules.
export TAG="crc-${VERSION}-${SHA}"
LABELS=${LABELS:-"latest crc-${VERSION}/crc-${VERSION}+latest"}

# Get the last release. We only create a new release if the main branch has moved since
# as trying to re-create an existing release is an error.
output=$(curl -fsS \
  -H "Accept: application/vnd.github+json" \
  -H "Authorization: token $GITHUB_TOKEN" \
  https://api.github.com/repos/$REPO/releases/latest)
PREVIOUS_RELEASE_NAME="$(jq -r '.tag_name'   <<< $output)"

if [ "$RELEASE_NAME" = "$PREVIOUS_RELEASE_NAME" ]; then
    echo "Release $RELEASE_NAME already exists. Nothing more to do."
    exit 0
else
    echo "Previous release is $PREVIOUS_RELEASE_NAME"
fi

CLOUD_ROBOTICS_CONTAINER_REGISTRY="gcr.io/cloud-robotics-releases"
# DOCKER_TAG is a global variable that is used in release_binary.
DOCKER_TAG=${DOCKER_TAG:-"crc-${VERSION}-${SHA}"}

# release_binary "${GCP_BUCKET}" "crc-${VERSION}/crc-${VERSION}+${SHA}" ${LABELS}

# Generate release notes comparing against the previous release.
output=$(curl -fsS \
  -X POST \
  -H "Accept: application/vnd.github+json" \
  -H "Authorization: token $GITHUB_TOKEN" \
  https://api.github.com/repos/$REPO/releases/generate-notes \
  -d '{"tag_name":"'$RELEASE_NAME'","previous_tag_name":"'$PREVIOUS_RELEASE_NAME'"}')
# Code newlines as literal \n and escape double quotes to make curl happy.
BODY="$(jq -r '.body'   <<< $output | awk '{printf "%s\\n", $0}' | sed 's/"/\\"/g')"
echo $BODY
echo "Generated release notes for $RELEASE_NAME"

# Create the release on GitHub.
curl -fsS \
  -X POST \
  -H "Accept: application/vnd.github+json" \
  -H "Authorization: token $GITHUB_TOKEN" \
  https://api.github.com/repos/$REPO/releases \
  --data-binary @- << EOF
{
  "tag_name": "$RELEASE_NAME",
  "name": "$RELEASE_NAME",
  "body": "$BODY"
}
EOF
