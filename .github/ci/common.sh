#!/bin/bash

# Format for the xtrace lines
export 'PS4=+$(date --rfc-3339=seconds):${BASH_SOURCE}:${LINENO}: '
set -o errexit   # exit immediately, if a pipeline command fails
set -o pipefail  # returns the last command to exit with a non-zero status
set -o xtrace    # print command traces before executing command

RUNFILES_ROOT="_main"

# Wraps the common Bazel flags for CI for brevity.
function bazel_ci {
  bazelisk --bazelrc="${DIR}/.bazelrc" "$@"
}

function generate_build_id() {
   # Considerations for a build identifier: It must be unique, it shouldn't break
   # if we try multiple dailies in a day, and it would be nice if a textual sort
   # would put newest releases last.
   git_hash=$(echo "$GITHUB_SHA" | cut -c1-6)
   date "+daily-%Y-%m-%d-${git_hash}"
}

# Pushes images and releases a binary to a specified bucket.
# bucket: target GCS bucket to release to
# name:  name of the release tar ball
# labels: optional list of filename aliases for the release, these are one-line
#   text files with the release name as a bucket local path
function release_binary {
  local bucket="$1"
  local name="$2"

  # This function is called from test and release pipelines. We (re)build the binary and push the
  # app images here to ensure the app images which are referenced in the binary exist in the
  # registry.
  bazel_ci build \
      //src/bootstrap/cloud:crc-binary \
      //src/app_charts:push \
      //src/go/cmd/setup-robot:setup-robot.push

  # The push scripts depends on binaries in the runfiles.
  local oldPwd
  oldPwd=$(pwd)
  # The tag variable must be called 'TAG', see cloud-robotics/bazel/container_push.bzl
  for t in latest ${DOCKER_TAG}; do
    cd ${oldPwd}/bazel-bin/src/go/cmd/setup-robot/push_setup-robot.push.sh.runfiles/${RUNFILES_ROOT}
    ${oldPwd}/bazel-bin/src/go/cmd/setup-robot/push_setup-robot.push.sh \
      --repository="${CLOUD_ROBOTICS_CONTAINER_REGISTRY}/setup-robot" \
      --tag="${t}"

    cd ${oldPwd}/bazel-bin/src/app_charts/push.runfiles/${RUNFILES_ROOT}
    TAG="$t" ${oldPwd}/bazel-bin/src/app_charts/push "${CLOUD_ROBOTICS_CONTAINER_REGISTRY}"
  done
  cd ${oldPwd}

  gcloud storage cp \
      --predefined-acl=publicRead \
      bazel-bin/src/bootstrap/cloud/crc-binary.tar.gz \
      "gs://${bucket}/${name}.tar.gz"

  # Overwrite cache control as we want changes to run-install.sh and version files to be visible
  # right away.
  gcloud storage cp \
      --predefined-acl=publicRead \
      --cache-control="private, max-age=0, no-transform" \
      src/bootstrap/cloud/run-install.sh \
      "gs://${bucket}/"

  # The remaining arguments are version labels. GCS does not support symlinks, so we use version
  # files instead.
  local vfile
  vfile=$(mktemp)
  echo "${name}.tar.gz" >${vfile}
  shift 2
  # Loop over remianing args in $* and creat alias files.
  for label; do
    gcloud storage cp \
        --predefined-acl=publicRead \
        --cache-control="private, max-age=0, no-transform" \
        ${vfile} "gs://${bucket}/${label}"
  done
}

# Populates a local cache mapping (tag, image, manifest_digest) for all top-level
# repositories in the given container registry using the Docker Registry v2 API.
# Note: The `.child` and `.manifest` fields in `/tags/list` responses are
# GCP-specific extensions supported by GCR and Google Artifact Registry.
# registry: container registry prefix (e.g. gcr.io/cloud-robotics-releases)
# cache_dir: local directory to store tag_index.tsv and cached manifests
function init_registry_tag_cache() {
  local registry="$1"
  local cache_dir="$2"

  if [[ -s "${cache_dir}/tag_index.tsv" ]]; then
    return 0
  fi

  local registry_host="${registry%%/*}"
  local registry_path="${registry#*/}"
  local base_url="https://${registry_host}/v2/${registry_path}"

  mkdir -p "${cache_dir}/manifests" "${cache_dir}/tags"

  # Write each repository's tag mappings to a separate file to avoid concurrent
  # writes to the same stream across parallel xargs workers.
  curl --fail-with-body -sS -4 "${base_url}/tags/list" \
    | jq -r '.child[]' \
    | xargs -P 8 -n 1 bash -c '
        set -euo pipefail
        base_url="$1"
        cache_dir="$2"
        image="$3"
        out_file="${cache_dir}/tags/${image//\//_}.tsv"
        curl --fail-with-body -sS -4 "${base_url}/${image}/tags/list" \
          | jq -r --arg image "${image}" '\''
              (.manifest // {}) | to_entries[]
              | .key as $digest
              | (.value.tag // [])[]
              | "\(.)\t\($image)\t\($digest)"
            '\'' > "${out_file}"
      ' _ "${base_url}" "${cache_dir}"

  cat "${cache_dir}"/tags/*.tsv > "${cache_dir}/tag_index.tsv.tmp"
  mv "${cache_dir}/tag_index.tsv.tmp" "${cache_dir}/tag_index.tsv"
}

# Outputs newline-delimited JSON rows of container image blob (config and layer)
# sizes for a single release tag.
# registry: container registry prefix (e.g. gcr.io/cloud-robotics-releases)
# tag: docker tag of the release (e.g. crc-0.1.0-<sha>)
# release_time: ISO-8601 timestamp for the release
# cache_dir: local directory to cache registry tag index and manifests
function collect_release_blob_rows() {
  local registry="$1"
  local tag="$2"
  local release_time="$3"
  local cache_dir="$4"

  init_registry_tag_cache "${registry}" "${cache_dir}"

  local registry_host="${registry%%/*}"
  local registry_path="${registry#*/}"
  local base_url="https://${registry_host}/v2/${registry_path}"

  local xtrace_was_set=false
  if [[ -o xtrace ]]; then
    xtrace_was_set=true
    set +o xtrace
  fi

  # Fetch any uncached manifests for this tag in parallel first.
  awk -F'\t' -v tag="${tag}" '$1 == tag {print $2 "\t" $3}' "${cache_dir}/tag_index.tsv" \
    | xargs -P 8 -n 2 bash -c '
        set -euo pipefail
        base_url="$1"
        cache_dir="$2"
        image="${3:-}"
        manifest_digest="${4:-}"
        if [[ -z "${image}" || -z "${manifest_digest}" ]]; then
          exit 0
        fi
        manifest_file="${cache_dir}/manifests/${manifest_digest//:/_}.json"
        if [[ ! -s "${manifest_file}" ]]; then
          curl --fail-with-body -sS -4 \
            -H "Accept: application/vnd.oci.image.manifest.v1+json, application/vnd.docker.distribution.manifest.v2+json" \
            "${base_url}/${image}/manifests/${manifest_digest}" > "${manifest_file}.tmp"
          mv "${manifest_file}.tmp" "${manifest_file}"
        fi
      ' _ "${base_url}" "${cache_dir}"

  local image manifest_digest manifest_file
  while IFS=$'\t' read -r image manifest_digest; do
    manifest_file="${cache_dir}/manifests/${manifest_digest//:/_}.json"
    jq -c \
      --arg tag "${tag}" \
      --arg release_time "${release_time}" \
      --arg image "${image}" \
      --arg manifest_digest "${manifest_digest}" \
      '
        ({
          release_tag: $tag,
          release_time: $release_time,
          image: $image,
          manifest_digest: $manifest_digest,
          blob_digest: .config.digest,
          blob_type: "config",
          size_bytes: .config.size
        }),
        ((.layers // [])[] | {
          release_tag: $tag,
          release_time: $release_time,
          image: $image,
          manifest_digest: $manifest_digest,
          blob_digest: .digest,
          blob_type: "layer",
          size_bytes: .size
        })
      ' "${manifest_file}"
  done < <(awk -F'\t' -v tag="${tag}" '$1 == tag {print $2 "\t" $3}' "${cache_dir}/tag_index.tsv")

  if [[ "${xtrace_was_set}" == "true" ]]; then
    set -o xtrace
  fi
}

# Collects container image blob sizes for a release tag and loads them into BigQuery,
# replacing any existing rows for the same release tag.
# registry: container registry prefix (e.g. gcr.io/cloud-robotics-releases)
# tag: docker tag of the release (e.g. crc-0.1.0-<sha>)
# release_time: optional ISO-8601 timestamp (defaults to current UTC time)
# bq_table: optional BigQuery table (defaults to cloud-robotics-releases:release_metrics.container_image_blobs)
# cache_dir: optional cache directory for reusing registry lookups across multiple releases
function publish_release_size_metrics() {
  local registry="$1"
  local tag="$2"
  local release_time
  release_time="${3:-$(date -u +"%Y-%m-%dT%H:%M:%SZ")}"
  local bq_table="${4:-cloud-robotics-releases:release_metrics.container_image_blobs}"
  local bq_project="${bq_table%%:*}"
  local cache_dir="${5:-}"
  local cleanup_cache=false

  if [[ -z "${cache_dir}" ]]; then
    cache_dir="$(mktemp -d)"
    cleanup_cache=true
  fi

  local rows_file
  rows_file="$(mktemp)"
  # shellcheck disable=SC2064
  trap "rm -f '${rows_file}'; if [[ '${cleanup_cache}' == 'true' ]]; then rm -rf '${cache_dir}'; fi" RETURN

  collect_release_blob_rows "${registry}" "${tag}" "${release_time}" "${cache_dir}" > "${rows_file}"

  if [[ -s "${rows_file}" ]]; then
    bq --project_id="${bq_project}" query --use_legacy_sql=false --quiet \
      --parameter="tag:STRING:${tag}" \
      "DELETE FROM \`${bq_table/:/\.}\` WHERE release_tag = @tag"
    bq --project_id="${bq_project}" load \
      --source_format=NEWLINE_DELIMITED_JSON \
      "${bq_table}" \
      "${rows_file}"
  else
    echo >&2 "Warning: No container images found in ${registry} for tag ${tag}"
  fi
}
