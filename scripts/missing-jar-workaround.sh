#!/bin/bash
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

# This script works around b/72145915 by invalidating the maven_jar
# repositories and rebuilding.

set -exo pipefail

cp third_party/maven_dependencies.bzl{,.bak}
function finish {
  mv third_party/maven_dependencies.bzl{.bak,}
}
trap finish EXIT

sed -i 's/\(sha1": "\)......../\1ffffffff/' third_party/maven_dependencies.bzl
! bazel build //third_party/java/... --keep_going

# Clear cache to make sure sources jars are present:
# https://github.com/bazelbuild/bazel/issues/4798
rm -rf .cache/content_addressable/sha1

cp third_party/maven_dependencies.bzl{.bak,}
bazel build //third_party/java/...
