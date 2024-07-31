#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "${DIR}/common.sh"
source "./scripts/common.sh"

# Because the format from common.sh is not recognized by Cloud Build.
export 'PS4='

# Need to source the project config from here
PROJECT_DIR="${DIR}/deployments/robco-integration-test"
source "${PROJECT_DIR}/config.sh"
gcloud config set project ${GCP_PROJECT_ID}
gke_get_credentials "${GCP_PROJECT_ID}" "cloud-robotics" "${GCP_REGION}" "${GCP_ZONE}"

BUILD_IDENTIFIER=$(generate_build_id)
echo "INFO: Build identifier is $BUILD_IDENTIFIER"

export BAZEL_FLAGS="--bazelrc=${DIR}/.bazelrc"
bash -x ./deploy.sh update "${GCP_PROJECT_ID}"

# Create a GKE cluster with a single robot for the relay test.
ROBOT_RELAY_CLUSTER="relay-test"
export SKIP_LOCAL_PULL=true
bash -x ./scripts/robot-sim.sh create "${GCP_PROJECT_ID}" "${ROBOT_RELAY_CLUSTER}"

bazel_ci run //src/go/cmd/setup-dev -- --project="${GCP_PROJECT_ID}" --robot-name="${ROBOT_RELAY_CLUSTER}"

DOMAIN=${CLOUD_ROBOTICS_DOMAIN:-"www.endpoints.${GCP_PROJECT_ID}.cloud.goog"}
ROBOT_CONTEXT="gke_${GCP_PROJECT_ID}_${GCP_ZONE}_${ROBOT_RELAY_CLUSTER}"

# Output state of cloud and robot k8s context to inspect the health of pods.
kubectl config get-contexts || true
kubectl --context ${CLOUD_ROBOTICS_CTX} get pods || true
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
