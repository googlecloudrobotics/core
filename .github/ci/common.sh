#!/bin/bash

# Format for the xtrace lines
export 'PS4=+$(date --rfc-3339=seconds):${BASH_SOURCE}:${LINENO}: '
set -o errexit   # exit immediately, if a pipeline command fails
set -o pipefail  # returns the last command to exit with a non-zero status
set -o xtrace    # print command traces before executing command

# Wraps the common Bazel flags for CI for brevity.
function bazel_ci {
  bazel --bazelrc="${DIR}/.bazelrc" "$@"
}

function generate_build_id() {
   # Considerations for a build identifier: It must be unique, it shouldn't break
   # if we try multiple dailies in a day, and it would be nice if a textual sort
   # would put newest releases last.
   git_hash=$(echo "$GITHUB_SHA" | cut -c1-6)
   date "+daily-%Y-%m-%d-${git_hash}"
}

function run_on_robot_sim() {
  local SIM_HOST="$1"
  shift
  # We don't know if this was executed with errexit on or off. Make sure that we
  # print the status and return the correct code either way.
  rc=0
  ssh -o "StrictHostKeyChecking=no" -i ~/.ssh/google_compute_engine builder@${SIM_HOST} "$@" || rc=$?
  echo "Done executing remote command: $* : ${rc}"
  return "${rc}"
}

function init_robot_sim() {
  local SIM_HOST="$1"
  local DEPLOY_FILES="$2"

  run_on_robot_sim ${SIM_HOST} 'rm -fr ~/robco/'

  echo "Uploading setup files"
  run_on_robot_sim ${SIM_HOST} "mkdir -p ~/robco"
  scp -o "StrictHostKeyChecking=no" -i ~/.ssh/google_compute_engine ${DEPLOY_FILES} ${SIM_HOST}:~/robco/

  # Terraform creates the robot-sim VM, but doesn't install the local cluster.
  # Since this script is idempotent, we run it on every test.
  # shellcheck disable=2088
  run_on_robot_sim ${SIM_HOST} "~/robco/install_k8s_on_robot.sh"
}

function cleanup_old_vm_instances() {
  # Aborted CI runs might leak VM instances, so we delete old tagged instances.
  local instances
  instances="$(gcloud compute instances list \
    --filter "tags.items=delete-after-one-day AND creationTimestamp<-P1D" \
    --project=${GCP_PROJECT_ID} --format='value(name)')"

  if [[ -n "$instances" ]] ; then
    gcloud compute instances delete $instances \
      --quiet --project=${GCP_PROJECT_ID} --zone=${GCP_ZONE}
  fi
}

function cleanup_old_ssh_keys() {
  # Work around overflowing the VM metadata store (b/113859328) - delete all past builder keys.
  local keys
  keys="$(mktemp /tmp/keys.XXXXXX)"

  gcloud compute project-info describe --format=json --project=${GCP_PROJECT_ID} | jq -r '.commonInstanceMetadata.items[] | select (.key == "ssh-keys") | .value' | egrep -v "^builder:" >${keys}
  gcloud compute project-info add-metadata --no-user-output-enabled --metadata-from-file ssh-keys=${keys} --project=${GCP_PROJECT_ID}
  rm -f ${keys}
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
  local oldPwd=$(pwd)
  # The tag variable must be called 'TAG', see cloud-robotics/bazel/container_push.bzl
  for t in latest ${DOCKER_TAG}; do
    cd ${oldPwd}/bazel-bin/src/go/cmd/setup-robot/push_setup-robot.push.sh.runfiles/cloud_robotics
    ${oldPwd}/bazel-bin/src/go/cmd/setup-robot/push_setup-robot.push.sh \
      --repository="${CLOUD_ROBOTICS_CONTAINER_REGISTRY}/setup-robot" \
      --tag="${t}"

    cd ${oldPwd}/bazel-bin/src/app_charts/push.runfiles/cloud_robotics
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


