#!/bin/bash
#
# Backfills historical release container image blob sizes into BigQuery by
# reading release timestamps from GCS and manifests from the container registry.

set -euo pipefail

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# shellcheck source=ci/common.sh
source "${DIR}/common.sh"

set +o xtrace

GCP_BUCKET=${GCP_BUCKET:-"cloud-robotics-releases"}
VERSION=${VERSION:-"0.1.0"}
CLOUD_ROBOTICS_CONTAINER_REGISTRY=${CLOUD_ROBOTICS_CONTAINER_REGISTRY:-"gcr.io/cloud-robotics-releases"}
BQ_TABLE=${BQ_TABLE:-"cloud-robotics-releases:release_metrics.container_image_blobs"}
BQ_PROJECT="${BQ_TABLE%%:*}"

CACHE_DIR="$(mktemp -d)"
EXISTING_TAGS_FILE="$(mktemp)"
ROWS_FILE="$(mktemp)"
trap 'rm -rf "${CACHE_DIR}" "${EXISTING_TAGS_FILE}" "${ROWS_FILE}"' EXIT

echo "Fetching existing release tags from BigQuery (${BQ_TABLE})..."
bq --project_id="${BQ_PROJECT}" query --use_legacy_sql=false --format=csv --max_rows=100000 \
  "SELECT DISTINCT release_tag FROM \`${BQ_TABLE/:/\.}\`" \
  | tail -n +2 > "${EXISTING_TAGS_FILE}" || true

echo "Building registry tag index from ${CLOUD_ROBOTICS_CONTAINER_REGISTRY}..."
init_registry_tag_cache "${CLOUD_ROBOTICS_CONTAINER_REGISTRY}" "${CACHE_DIR}"

echo "Listing historical releases from gs://${GCP_BUCKET}/crc-${VERSION}/..."
while IFS=$'\t' read -r tag release_time; do
  if grep -Fxq "${tag}" "${EXISTING_TAGS_FILE}"; then
    echo "Skipping ${tag} (already in BigQuery)"
    continue
  fi
  echo "Collecting metrics for ${tag} (${release_time})..."
  collect_release_blob_rows \
    "${CLOUD_ROBOTICS_CONTAINER_REGISTRY}" \
    "${tag}" \
    "${release_time}" \
    "${CACHE_DIR}" >> "${ROWS_FILE}"
done < <(
  gcloud storage ls --long "gs://${GCP_BUCKET}/crc-${VERSION}/crc-${VERSION}+*.tar.gz" \
    | awk '
        $3 ~ /\.tar\.gz$/ {
          release_time = $2
          tag = $3
          sub(/^.*\/crc-/, "crc-", tag)
          sub(/\.tar\.gz$/, "", tag)
          sub(/\+/, "-", tag)
          print tag "\t" release_time
        }
      '
)

if [[ -s "${ROWS_FILE}" ]]; then
  echo "Loading collected rows into BigQuery (${BQ_TABLE})..."
  bq --project_id="${BQ_PROJECT}" load \
    --source_format=NEWLINE_DELIMITED_JSON \
    "${BQ_TABLE}" \
    "${ROWS_FILE}"
else
  echo "No new releases to backfill."
fi
