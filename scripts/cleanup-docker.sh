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

# Remove stale/old docker images
#
# Run as:
# ./cleanup-docker.sh local_cleanup
# ./cleanup-docker.sh cloud_cleanup <project-id>

function local_cleanup {
  # TODO(ensonic) in docker 1.13.X we can use: docker container prune and
  # docker system prune

  # Delete stopped containers
  containers=$(docker ps --filter status=exited -q)
  test -n "$containers" && docker rm -v $containers
  # Delete created (and never executed) containers
  containers=$(docker ps --filter status=created -q)
  test -n "$containers" && docker rm -v $containers
  # Delete unused images
  images=$(docker images --filter dangling=true -q)
  test -n "$images" && docker rmi $images
}

function cloud_cleanup {
  PROJECT=${1:-$(gcloud 2>/dev/null config get-value project)}

  # Unique image names
  names=$(gcloud container images list --repository=eu.gcr.io/${PROJECT} --format='value(name)')
  for name in $names; do
    # TODO(ensonic): the images we build all have TIMESTAMP=1970-01-01T01:00:00
    # hence we delete all but the one tagged 'latest'
    digests=$(gcloud container images list-tags $name --format='get(digest)' --filter='-tags:*')
    for digest in $digests; do
      gcloud container images delete --force-delete-tags -q ${name}@${digest}
    done
  done
}

# call arguments verbatim:
$@
