#!/bin/bash
# Fetches the cert-manager chart for the version in BUILD.bazel, replacing the
# previously vendored chart.
#
# Usage: third_party/cert-manager/update.sh [VERSION]
#
# If VERSION is given, CERT_MANAGER_VERSION in BUILD.bazel is updated first.
# Renovate updates BUILD.bazel itself and then runs this script without
# arguments, see .github/ci/renovate-config.js.

set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD="${DIR}/BUILD.bazel"

if [[ $# -gt 1 ]]; then
  echo "Usage: $0 [VERSION]" >&2
  exit 1
elif [[ $# -eq 1 ]]; then
  sed -i "s/^CERT_MANAGER_VERSION = \".*\"$/CERT_MANAGER_VERSION = \"$1\"/" "${BUILD}"
fi

version="$(sed -n 's/^CERT_MANAGER_VERSION = "\(.*\)"$/\1/p' "${BUILD}")"
if [[ -z "${version}" ]]; then
  echo "CERT_MANAGER_VERSION not found in ${BUILD}" >&2
  exit 1
fi

rm -f "${DIR}"/cert-manager-*.tgz
helm pull cert-manager --repo https://charts.jetstack.io --version "${version}" -d "${DIR}"
