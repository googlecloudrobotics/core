#!/bin/bash

# Format for the xtrace lines
export 'PS4=+$(date --rfc-3339=seconds):${BASH_SOURCE}:${LINENO}: '
set -o errexit   # exit immediately, if a pipeline command fails
set -o pipefail  # returns the last command to exit with a non-zero status
set -o xtrace    # print command traces before executing command

RUNFILES_ROOT="_main"

# Wraps the common Bazel flags for CI for brevity.
function bazel_ci {
  bazelisk --bazelrc="${DIR}/.bazelrc" "$@"
}

function generate_build_id() {
   # Considerations for a build identifier: It must be unique, it shouldn't break
   # if we try multiple dailies in a day, and it would be nice if a textual sort
   # would put newest releases last.
   git_hash=$(echo "$GITHUB_SHA" | cut -c1-6)
   date "+daily-%Y-%m-%d-${git_hash}"
}

# Pushes images and releases a binary to a specified bucket.
# bucket: target GCS bucket to release to
# name:  name of the release tar ball
# labels: optional list of filename aliases for the release, these are one-line
#   text files with the release name as a bucket local path
function release_binary {
  local bucket="$1"
  local name="$2"

  # This function is called from test and release pipelines. We (re)build the binary and push the
  # app images here to ensure the app images which are referenced in the binary exist in the
  # registry.
  bazel_ci build \
      //src/bootstrap/cloud:crc-binary \
      //src/app_charts:push \
      //src/go/cmd/setup-robot:setup-robot.push

  # The push scripts depends on binaries in the runfiles.
  local oldPwd
  oldPwd=$(pwd)
  # The tag variable must be called 'TAG', see cloud-robotics/bazel/container_push.bzl
  for t in latest ${DOCKER_TAG}; do
    cd ${oldPwd}/bazel-bin/src/go/cmd/setup-robot/push_setup-robot.push.sh.runfiles/${RUNFILES_ROOT}
    ${oldPwd}/bazel-bin/src/go/cmd/setup-robot/push_setup-robot.push.sh \
      --repository="${CLOUD_ROBOTICS_CONTAINER_REGISTRY}/setup-robot" \
      --tag="${t}"

    cd ${oldPwd}/bazel-bin/src/app_charts/push.runfiles/${RUNFILES_ROOT}
    TAG="$t" ${oldPwd}/bazel-bin/src/app_charts/push "${CLOUD_ROBOTICS_CONTAINER_REGISTRY}"
  done
  cd ${oldPwd}

  gsutil cp -a public-read \
      bazel-bin/src/bootstrap/cloud/crc-binary.tar.gz \
      "gs://${bucket}/${name}.tar.gz"

  # Overwrite cache control as we want changes to run-install.sh and version files to be visible
  # right away.
  gsutil -h "Cache-Control:private, max-age=0, no-transform" \
      cp -a public-read \
      src/bootstrap/cloud/run-install.sh \
      "gs://${bucket}/"

  # The remaining arguments are version labels. gsutil does not support symlinks, so we use version
  # files instead.
  local vfile
  vfile=$(mktemp)
  echo "${name}.tar.gz" >${vfile}
  shift 2
  # Loop over remianing args in $* and creat alias files.
  for label; do
    gsutil -h "Cache-Control:private, max-age=0, no-transform" \
        cp -a public-read \
        ${vfile} "gs://${bucket}/${label}"
  done
}


