#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "${DIR}/common.sh"

LOCK_OBJECT=gs://robco-integration-test-lock/lock
LOCK_BACKOFF_SECONDS=60

lock() {
  # Take the lock by creating the lock object. x-goog-if-generation-match:0 is a
  # GCS precondition that causes `cp` to fail if the lock object already exists.
  while ! echo "lock" | gsutil -q -h "x-goog-if-generation-match:0" cp - $LOCK_OBJECT
  do
    : "lock: failed to obtain lock, retrying in $LOCK_BACKOFF_SECONDS seconds"
    : "Note to build cop: if you think there is a stale lock, run:"
    : "    gsutil rm $LOCK_OBJECT"
    : "This can occur when a previous job timed out or was canceled while"
    : "holding the lock."
    sleep $LOCK_BACKOFF_SECONDS
  done
  # TODO(rodrigoq): if the build is cancelled by GitHub, the lock is not
  # released. The GCS lifecycle will delete the lock after a day, if the build
  # cop doesn't delete it sooner. We could add a check here to delete the lock
  # if it's too old, but I don't know how to do that safely - maybe a second
  # lock would prevent races between deletion checks, but maybe it would just
  # introduce other failure modes.
}

finalize_and_unlock() {
  local sleep_time=1
  while ! gsutil -q rm $LOCK_OBJECT
  do
    echo "unlock: failed to relinquish lock, retrying in $sleep_time seconds"
    sleep $sleep_time
    sleep_time=$(expr $sleep_time '*' 2)
  done
}

# Need to source the project config from here
PROJECT_DIR="${DIR}/deployments/robco-integration-test"
source "${PROJECT_DIR}/config.sh"

BUILD_IDENTIFIER=$(generate_build_id)
echo "INFO: Build identifier is $BUILD_IDENTIFIER"

# Get the lock before deploying to the project. This ensures that other runs
# will not change our deployment until we finish testing.
lock

# `set +x` avoids log spam and makes error messages more obvious.
trap 'set +x; finalize_and_unlock' EXIT

# Create a GKE cluster with a single robot for the relay test.
ROBOT_RELAY_CLUSTER="relay-test"
bash -x ./scripts/robot-sim.sh create "${GCP_PROJECT_ID}" "${ROBOT_RELAY_CLUSTER}"

export BAZEL_FLAGS="--bazelrc=${DIR}/.bazelrc"
bash -x ./deploy.sh update "${GCP_PROJECT_ID}"

DOMAIN=${CLOUD_ROBOTICS_DOMAIN:-"www.endpoints.${GCP_PROJECT_ID}.cloud.goog"}
CLOUD_CONTEXT="gke_${GCP_PROJECT_ID}_${GCP_ZONE}_cloud-robotics"
ROBOT_CONTEXT="gke_${GCP_PROJECT_ID}_${GCP_ZONE}_${ROBOT_RELAY_CLUSTER}"

# Output state of cloud and robot k8s context to inspect the health of pods.
kubectl config get-contexts || true
kubectl --context ${CLOUD_CONTEXT} get pods || true
kubectl --context ${GCP_PROJECT_ID}-robot get pods || true
kubectl --context ${ROBOT_CONTEXT} get pods || true

bazel_ci test \
  --test_env GCP_PROJECT_ID=${GCP_PROJECT_ID} \
  --test_env GCP_REGION=${GCP_REGION} \
  --test_env GCP_ZONE=${GCP_ZONE} \
  --test_env CLUSTER=${ROBOT_RELAY_CLUSTER} \
  --test_env PATH=$PATH \
  --jvmopt="-DCLOUD_ROBOTICS_DOMAIN=${DOMAIN}" \
  --test_output=streamed \
  --test_tag_filters="external" \
  --strategy=TestRunner=standalone \
  //...

# If this is running on main (ie, not a manual run) then update the `latest`
# binary.
if [[ "$MANUAL_RUN" == "false" ]] ; then
  release_binary "robco-ci-binary-builds" "crc-${BUILD_IDENTIFIER}" "latest"
fi
