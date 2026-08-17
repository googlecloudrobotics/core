#!/bin/bash

VERSION=1.29.4

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

tmpdir="$(mktemp -d)"
trap "rm -rf '${tmpdir}'" EXIT
echo "Downloading istioctl ${VERSION}..."
curl -fsSl "https://storage.googleapis.com/istio-release/releases/${VERSION}/istioctl-${VERSION}-linux-amd64.tar.gz" \
  | tar -C "${tmpdir}" -zx istioctl
istioctl="${tmpdir}/istioctl"
# for faster testing
#istioctl="${HOME}/bin/istioctl"

if [[ ! -x "${istioctl}" ]] ; then
  echo "Failed to extract istioctl from tarball." >&2
  exit 1
fi

# Istio needs to be able to check the Kubernetes version, otherwise it targets
# 1.20 which is very old. It doesn't need to be identical to the target system,
# just not too far off that we have compatibility issues.
if ! kubectl version --output=yaml "$@" >/dev/null; then
  echo "Failed to check kubernetes version." >&2
  echo "Try using --context=minikube or another valid context." >&2
  exit 1
fi

echo "Updating to istio $("${istioctl}" version --remote=false)..."

# Step 1: Generate Istio YAML.  See istio_operator.yaml for our tweaks to the
# default profile.
# https://istio.io/latest/docs/setup/additional-setup/customize-installation/

"${istioctl}" manifest generate \
  -f "${SCRIPT_DIR}/istio_operator.yaml" \
  --cluster-specific \
  "$@" > ${tmpdir}/istio_full.yaml

# Step 2: Process Istio manifests with python helper
dst="${SCRIPT_DIR}/istio-generated.yaml"
crd_dst="${SCRIPT_DIR}/istio-crds.yaml"
python3 "${SCRIPT_DIR}/process_istio_manifest.py" \
  "${tmpdir}/istio_full.yaml" \
  "${SCRIPT_DIR}/istio-config.yaml" \
  "${SCRIPT_DIR}/istio-values.json" \
  "${tmpdir}/istio.yaml" \
  "${dst}" \
  "${crd_dst}"

echo "Updated ${dst} and ${crd_dst}"

# Step 3: Download and save Istio Grafana dashboards
echo "Downloading Istio Grafana dashboards..."
DASHBOARDS=(
  "istio-mesh-dashboard.gen.json"
  "istio-performance-dashboard.json"
  "istio-service-dashboard.json"
  "istio-workload-dashboard.json"
  "pilot-dashboard.gen.json"
)

curl_args=()
for dashboard in "${DASHBOARDS[@]}"; do
  curl_args+=(
    -o "${tmpdir}/${dashboard}"
    "https://raw.githubusercontent.com/istio/istio/${VERSION}/manifests/addons/dashboards/${dashboard}"
  )
done

if ! curl -fsSL "${curl_args[@]}"; then
  echo "Failed to download dashboards" >&2
  exit 1
fi

DASHBOARD_DIR="${SCRIPT_DIR}/grafana-dashboards"
mkdir -p "${DASHBOARD_DIR}"

for dashboard in "${DASHBOARDS[@]}"; do
  json_file="${tmpdir}/${dashboard}"
  name=$(basename "${dashboard}" | sed -E 's/\.gen\.json$//' | sed -E 's/\.json$//')
  key="${name}.json"
  cp "${json_file}" "${DASHBOARD_DIR}/${key}"
done

# Step 4: Verify generated YAML syntax
echo "Verifying generated YAML syntax..."
if ! python3 "${SCRIPT_DIR}/validate_istio_manifest.py" "${dst}"; then
  echo "Error: ${dst} failed YAML validation!" >&2
  exit 1
fi

echo "Verifying base-cloud chart build (helm lint)..."
if ! bazel build //src/app_charts/base:base-cloud; then
  echo "Error: base-cloud chart failed helm lint / bazel build validation!" >&2
  exit 1
fi

