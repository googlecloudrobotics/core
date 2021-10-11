#!/usr/bin/env bash
#
# Copyright 2021 The Cloud Robotics Authors
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

# This script updates the Go dependencies in go.mod and go.sum and applies
# changes to the WORKSPACE file.
set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

export GO111MODULE=on

cd ${DIR}
# TODO(ensonic): how to resolve local imports to proto genenrated go code
go mod tidy || /bin/true
bazel run //:gazelle -- update-repos -from_file=${DIR}/go.mod -to_macro=third_party/go_repositories.bzl%go_repositories -prune
bazel run //:gazelle

echo "updates done"

