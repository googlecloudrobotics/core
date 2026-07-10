#!/bin/bash
# Print workspace status variables for Bazel stamping.

# Plumb BUILD_VERSION env var (from release_binary.sh) to the Helm chart.
#
# "STABLE_" doesn't refer to the build, but rather that the value won't change
# during the build, so Bazel should always use the right value:
# https://bazel.build/docs/user-manual#workspace-status
echo "STABLE_BUILD_VERSION ${BUILD_VERSION:-0.1.0-dev}"
