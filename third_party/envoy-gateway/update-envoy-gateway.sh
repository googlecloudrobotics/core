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

# Check Envoy Gateway, Envoy Proxy, Gateway API, and Kubernetes version compatibility at:
# https://gateway.envoyproxy.io/news/releases/matrix/
EG_VERSION="${EG_VERSION:-v1.8.3}"
GAPI_VERSION="${GAPI_VERSION:-v1.5.1}"
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
CORE_ROOT=$( cd -- "${SCRIPT_DIR}/../.." &> /dev/null && pwd )

echo "Downloading official Gateway API Standard Channel ${GAPI_VERSION} CRDs..."
gapi_dst="${CORE_ROOT}/third_party/gateway-api/gatewayapi-crds.yaml"
curl -fsSL "https://github.com/kubernetes-sigs/gateway-api/releases/download/${GAPI_VERSION}/standard-install.yaml" -o "${gapi_dst}"

echo "Downloading official Envoy Gateway ${EG_VERSION} install manifest..."

tmpdir="$(mktemp -d)"
trap 'rm -rf "${tmpdir}"' EXIT

install_manifest="${tmpdir}/install.yaml"
curl -fsSL "https://github.com/envoyproxy/gateway/releases/download/${EG_VERSION}/install.yaml" -o "${install_manifest}"

eg_crd_dst="${SCRIPT_DIR}/envoy-gateway-crds.yaml"
eg_manifest_dst="${SCRIPT_DIR}/envoy-gateway-generated.yaml"

echo "Processing Envoy Gateway and Gateway API manifests..."
python3 "${SCRIPT_DIR}/process_envoy_gateway_manifest.py" \
  "${install_manifest}" \
  "${gapi_dst}" \
  "${eg_crd_dst}" \
  "${eg_manifest_dst}"

echo "Envoy Gateway (${EG_VERSION}) and Gateway API (${GAPI_VERSION}) assets updated successfully!"
