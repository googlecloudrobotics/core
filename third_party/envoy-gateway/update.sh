#!/bin/bash
#
# Copyright 2026 The Cloud Robotics Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -euo pipefail

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
CORE_ROOT=$( cd -- "${SCRIPT_DIR}/../.." &> /dev/null && pwd )

eg_override="${EG_VERSION:-}"

# Check Envoy Gateway, Envoy Proxy, Gateway API, and Kubernetes version compatibility at:
# https://gateway.envoyproxy.io/news/releases/matrix/
# Updated by Renovate (see .github/ci/renovate-config.js). To update manually, run:
#   EG_VERSION=<version> third_party/envoy-gateway/update.sh
EG_VERSION="${EG_VERSION:-v1.8.3}"
if [[ -n "${eg_override}" ]]; then
  sed -i "s/^EG_VERSION=\"\\\${EG_VERSION:-.*}\"$/EG_VERSION=\"\${EG_VERSION:-${EG_VERSION}}\"/" \
    "${SCRIPT_DIR}/update.sh"
fi

if [[ -z "${GAPI_VERSION:-}" ]]; then
  echo "Resolving compatible Gateway API version from Envoy Gateway ${EG_VERSION} go.mod..."
  gomod="$(curl -fsSL "https://raw.githubusercontent.com/envoyproxy/gateway/${EG_VERSION}/go.mod")"
  GAPI_VERSION="$(
    awk '$1 == "sigs.k8s.io/gateway-api" { print $2; exit } $1 == "require" && $2 == "sigs.k8s.io/gateway-api" { print $3; exit }' <<< "${gomod}"
  )"
  if [[ -z "${GAPI_VERSION}" ]]; then
    echo "Failed to resolve sigs.k8s.io/gateway-api version for Envoy Gateway ${EG_VERSION}" >&2
    exit 1
  fi
fi
export GAPI_VERSION

sed -i "s/^GAPI_VERSION=\"\\\${GAPI_VERSION:-.*}\"$/GAPI_VERSION=\"\${GAPI_VERSION:-${GAPI_VERSION}}\"/" \
  "${CORE_ROOT}/third_party/gateway-api/update.sh"

# Invoke the split-out Gateway API update script
"${CORE_ROOT}/third_party/gateway-api/update.sh"

echo "Downloading official Envoy Gateway ${EG_VERSION} install manifest..."

tmpdir="$(mktemp -d)"
trap 'rm -rf "${tmpdir}"' EXIT

install_manifest="${tmpdir}/install.yaml"
curl -fsSL "https://github.com/envoyproxy/gateway/releases/download/${EG_VERSION}/install.yaml" -o "${install_manifest}"

eg_crd_dst="${SCRIPT_DIR}/envoy-gateway-crds.yaml"
eg_manifest_dst="${SCRIPT_DIR}/envoy-gateway-generated.yaml"

echo "Processing Envoy Gateway manifests..."
python3 "${SCRIPT_DIR}/process_envoy_gateway_manifest.py" \
  "${install_manifest}" \
  "${eg_crd_dst}" \
  "${eg_manifest_dst}"

dashboards_dst="${SCRIPT_DIR}/dashboards"
mkdir -p "${dashboards_dst}"
echo "Downloading official Envoy Gateway ${EG_VERSION} Grafana dashboards..."
curl -fsSL "https://raw.githubusercontent.com/envoyproxy/gateway/${EG_VERSION}/charts/gateway-addons-helm/dashboards/envoy-proxy-global.json" -o "${dashboards_dst}/envoy-proxy-global.json"
curl -fsSL "https://raw.githubusercontent.com/envoyproxy/gateway/${EG_VERSION}/charts/gateway-addons-helm/dashboards/envoy-gateway-global.json" -o "${dashboards_dst}/envoy-gateway-global.json"
curl -fsSL "https://raw.githubusercontent.com/envoyproxy/gateway/${EG_VERSION}/charts/gateway-addons-helm/dashboards/envoy-clusters.json" -o "${dashboards_dst}/envoy-clusters.json"

sed -i -E \
  -e "s|^(- \*\*Envoy Gateway\*\*: )\[[^]]+\]\([^)]+\)|\1[${EG_VERSION}](https://github.com/envoyproxy/gateway/releases/tag/${EG_VERSION})|" \
  -e "s|^(- \*\*Gateway API\*\*: )\[[^]]+\]\([^)]+\)|\1[${GAPI_VERSION}](https://github.com/kubernetes-sigs/gateway-api/releases/tag/${GAPI_VERSION})|" \
  -e "s|^EG_VERSION=[^ ]+ GAPI_VERSION=[^ ]+ ./update\.sh$|EG_VERSION=${EG_VERSION} GAPI_VERSION=${GAPI_VERSION} ./update.sh|" \
  "${SCRIPT_DIR}/README.md"

echo "Envoy Gateway (${EG_VERSION}) and Gateway API (${GAPI_VERSION}) assets updated successfully!"
