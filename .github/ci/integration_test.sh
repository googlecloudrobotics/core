#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "${DIR}/common.sh"

# Because the format from common.sh is not recognized by Cloud Build.
export 'PS4='

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
  # Clean up CR of test robot.
  kubectl delete robots.registry.cloudrobotics.com "${NEW_ROBOT_NAME}" &> /dev/null || true

  cleanup_old_ssh_keys || true
  cleanup_old_vm_instances || true

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
gcloud config set project ${GCP_PROJECT_ID}
gcloud container clusters get-credentials cloud-robotics --zone=${GCP_ZONE}

BUILD_IDENTIFIER=$(generate_build_id)
echo "INFO: Build identifier is $BUILD_IDENTIFIER"

bazel_ci build //...

# Get the lock before deploying to the project. This ensures that other runs
# will not change our deployment until we finish testing.
lock

# `set +x` avoids log spam and makes error messages more obvious.
trap 'set +x; finalize_and_unlock' EXIT

export BAZEL_FLAGS="--bazelrc=${DIR}/.bazelrc"
bash -x .//deploy.sh update robco-integration-test

DOMAIN=${CLOUD_ROBOTICS_DOMAIN:-"www.endpoints.${GCP_PROJECT_ID}.cloud.goog"}
CLOUD_CONTEXT="gke_${GCP_PROJECT_ID}_${GCP_ZONE}_cloud-robotics"
SETUP_DEV_BINARY=./bazel-bin/src/go/cmd/setup-dev/setup-dev_/setup-dev

# This generates a .ssh/config for the sim-host
gcloud compute config-ssh

# The `name` here should match the instance name in
# ci/terraform/robco-integration-test.sh.
# The `|| true` and `if [[ -z ...` bits work around a gcloud issue (b/147795223).
SIM_HOST="$(gcloud compute instances list --project ${GCP_PROJECT_ID} --filter='name=("robot-sim")' --format='value(networkInterfaces.networkIP)' || true)"
if [[ -z "$SIM_HOST" ]] ; then
  echo "Failed to get IP of robot-sim VM instance." >&2
  exit 1
fi

# BUG(https://github.com/googlecloudrobotics/core/issues/346)
# install_k8s_on_robot is currently broken. The rest of the test will work
# for as long as our old VM is functional.
# DEPLOY_FILES="src/bootstrap/robot/setup_robot.sh \
#   src/bootstrap/robot/install_k8s_on_robot.sh \
#   ./bazel-out/../../../external/kubernetes_helm/helm"
# init_robot_sim ${SIM_HOST} "${DEPLOY_FILES}"

# Setup new robot
NEW_ROBOT_NAME="test-robot"
NEW_ROBOT_TYPE="test-robot-type"

# Pre-create metadata-server firewall rule to avoid race (b/121175402).
METADATA_SERVER_RULE="-p tcp -d 169.254.169.254 --dport 80 -j DNAT --to-destination 127.0.0.1:8965 -m comment --comment 'from ci/integration_test.sh'"
run_on_robot_sim ${SIM_HOST} \
  "sudo iptables --table nat --wait --verbose --check PREROUTING ${METADATA_SERVER_RULE} \
  || sudo iptables --table nat --wait --verbose --append PREROUTING ${METADATA_SERVER_RULE}"

gcloud auth application-default print-access-token --project ${GCP_PROJECT_ID} | \
  run_on_robot_sim ${SIM_HOST} "cat > ~/access_token"
run_on_robot_sim ${SIM_HOST} "ACCESS_TOKEN_FILE=~/access_token ~/robco/setup_robot.sh ${NEW_ROBOT_NAME} --project ${GCP_PROJECT_ID} --robot-type ${NEW_ROBOT_TYPE}" || {
  : "setup_robot failed."
  : "If you see 'certificate has expired or is not yet valid' above (b/178455122), try:"
  : "  gcloud compute config-ssh --project=robco-integration-test"
  : "  ssh robot-sim.europe-west1-c.robco-integration-test"
  : "  sudo kubeadm reset --force"
  exit 1
}
run_on_robot_sim ${SIM_HOST} "rm ~/access_token"

# TODO(b/121119919): remove this workaround
run_on_robot_sim ${SIM_HOST} "kubectl delete pod -l name=metadata-server"
# TODO(b/153142491): remove this workaround
run_on_robot_sim ${SIM_HOST} "kubectl delete pod -l app=gcr-credential-refresher"

"${SETUP_DEV_BINARY}" --project="${GCP_PROJECT_ID}" --robot-name="${NEW_ROBOT_NAME}"

# Deploy the k8s relay rollout.
kubectl apply -f "${DIR}/deployments/robco-integration-test/kubernetes/"

# Output state of cloud and robot k8s context to inspect the health of pods.
kubectl config get-contexts || true
kubectl --context ${CLOUD_CONTEXT} get pods || true
kubectl --context ${GCP_PROJECT_ID}-robot get pods || true

# For some reason //src/go/tests:go_default_test is expecting
# the kubeconfig in /home/builder/.kube/config, i.e. it does not use $HOME
# (which is /builder/home). alexanderfaxa@ could not figure out why so just
# copy the config there.
mkdir -p /home/builder/.kube
cp /builder/home/.kube/config /home/builder/.kube/config

bazel_ci test \
  --test_env GCP_PROJECT_ID=${GCP_PROJECT_ID} \
  --test_env GCP_REGION=${GCP_REGION} \
  --test_env GCP_ZONE=${GCP_ZONE} \
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
