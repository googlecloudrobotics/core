#!/usr/bin/env bash
#
# Copyright 2019 The Google Cloud Robotics Authors
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

CURRENT_DIR=$(pwd)
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# We create the shadow repo one dir up because the dep tool falsely tries to
# truncate the GOPATH we provide after the first /go/ dir it sees.
SHADOW_REPO="${DIR}/../.gopath/src/cloud-robotics.googlesource.com/cloud-robotics"

export GOPATH="${DIR}/../.gopath"
go get k8s.io/code-generator/...

export PATH="$PATH:$GOPATH/bin"
mkdir -p ${SHADOW_REPO}

rm -rf "${DIR}/pkg/client"
rm -rf "${SHADOW_REPO}/pkg/apis"
rm -rf "${SHADOW_REPO}/pkg/client"
cp -r ${DIR}/* ${SHADOW_REPO}

function finalize {
  cp -rT ${SHADOW_REPO}/pkg/client ${DIR}/pkg/client
  cp -rT ${SHADOW_REPO}/pkg/apis ${DIR}/pkg/apis
  cd ${CURRENT_DIR}
}

trap finalize EXIT
cd ${SHADOW_REPO}

REPO=cloud-robotics.googlesource.com/cloud-robotics

cat > "${SHADOW_REPO}/HEADER" <<EOF
// Copyright $(date +%Y) The Google Cloud Robotics Authors
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

deepcopy-gen \
  --go-header-file   "${SHADOW_REPO}/HEADER" \
  --input-dirs       "${REPO}/pkg/apis/apps/v1alpha1" \
  --bounding-dirs    "${REPO}/pkg/apis/apps" \
  --output-file-base zz_generated.deepcopy

client-gen \
  --go-header-file "${SHADOW_REPO}/HEADER" \
  --input-base     "${REPO}/pkg/apis" \
  --clientset-name "versioned" \
  --input          "/apps/v1alpha1" \
  --output-package "${REPO}/pkg/client" \

lister-gen \
  --go-header-file "${SHADOW_REPO}/HEADER" \
  --input-dirs     "${REPO}/pkg/apis/apps/v1alpha1" \
  --output-package "${REPO}/pkg/client/listers"

informer-gen \
  --go-header-file   "${SHADOW_REPO}/HEADER" \
  --single-directory \
  --listers-package  "${REPO}/pkg/client/listers" \
  --input-dirs       "${REPO}/pkg/apis/apps/v1alpha1" \
  --output-package   "${REPO}/pkg/client/informers" \
  --versioned-clientset-package "${REPO}/pkg/client/versioned"

# Re-generate BUILD files for generated packages.
bazel run //:gazelle
