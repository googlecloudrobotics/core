#!/usr/bin/env bash
#
# Copyright 2019 The Cloud Robotics Authors
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

# This script can be run just like the regular dep tool. It copies the Go
# code to a shadow repo against dep can operate as usual and copies the
# resulting Gopkg.toml and Gopkg.lock files to this directory.
# It then stages the changed dependenies in the bazel WORKSPACE for manual cleanup.

set -e

# K8S release for api, apimachinery and code-generator
K8S_RELEASE="v0.37.1"

CURRENT_DIR=$(pwd)
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

export GOPATH="${DIR}/../.gopath"
export GOBIN="${GOPATH}/bin"

go install k8s.io/code-generator/cmd/{applyconfiguration-gen,defaulter-gen,client-gen,lister-gen,informer-gen,deepcopy-gen}@${K8S_RELEASE}

export PATH="$PATH:$GOPATH/bin"

rm -rf "${DIR}/pkg/client"

HEADER="${GOPATH}/HEADER"

function finalize {
  rm -f "${HEADER}"
  cd ${CURRENT_DIR}

  # Re-generate BUILD files for generated packages.
  ${DIR}/../../gomod.sh
}

trap finalize EXIT
cd ${DIR}

REPO=github.com/googlecloudrobotics/core/src/go

cat > "${HEADER}" <<EOF
// Copyright $(date +%Y) The Cloud Robotics Authors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
EOF

dirs=()
groupversions=""

for d in ${DIR}/pkg/apis/*/*; do
  version=$(basename $d)
  group=$(basename "$(dirname $d)")
  echo "generating for ${group}/${version}"

  groupversions="${groupversions},${group}/${version}"
  dirs+=("${REPO}/pkg/apis/${group}/${version}")
done

groupversions="${groupversions:1}"

${GOBIN}/deepcopy-gen \
  --go-header-file "${HEADER}" \
  --output-file    zz_generated.deepcopy.go \
  "${dirs[@]}"

${GOBIN}/client-gen \
  --go-header-file "${HEADER}" \
  --clientset-name "versioned" \
  --input-base     "${REPO}/pkg/apis" \
  --input          "${groupversions}" \
  --output-dir     "${DIR}/pkg/client" \
  --output-pkg     "${REPO}/pkg/client"

${GOBIN}/lister-gen \
  --go-header-file "${HEADER}" \
  --output-dir     "${DIR}/pkg/client/listers" \
  --output-pkg     "${REPO}/pkg/client/listers" \
  "${dirs[@]}"

${GOBIN}/informer-gen \
  --go-header-file              "${HEADER}" \
  --single-directory \
  --listers-package             "${REPO}/pkg/client/listers" \
  --output-dir                  "${DIR}/pkg/client/informers" \
  --output-pkg                  "${REPO}/pkg/client/informers" \
  --versioned-clientset-package "${REPO}/pkg/client/versioned" \
  "${dirs[@]}"
