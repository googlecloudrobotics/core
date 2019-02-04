#!/usr/bin/env bash

set -eu

function guess_runfiles() {
    pushd ${BASH_SOURCE[0]}.runfiles > /dev/null 2>&1
    pwd
    popd > /dev/null 2>&1
}

RUNFILES="${PYTHON_RUNFILES:-$(guess_runfiles)}"

# app() uses run_parallel() to push images to GCR, which relies on
# gcloud to get credentials. That, however, has a race condition:
# https://github.com/google/containerregistry/issues/115
# As such, we cache credentials and create a script that prints them to
# replace the racy credential helper. This script is added to to the start of
# PATH. This is harmless if run_parallel() is being used for something else.
# It also saves ~5s of CPU time.
tmp_bin=$(mktemp --tmpdir= -d deploy-XXXXXXXX-bin)
export PATH="${tmp_bin}:${PATH}"
function rm_tmp_bin {
  rm -r "${tmp_bin}"
}
trap rm_tmp_bin EXIT

credential_script=$tmp_bin/docker-credential-gcloud
credential_file=$tmp_bin/docker-credential-gcloud.json
gcp_registry=$(echo "%{gcr_registry}" | cut -d'/' -f 1)
docker-credential-gcloud get <<<"https://${gcp_registry}" > "${credential_file}"
cat > "${credential_script}" << EOF
#!/bin/bash
cat "${credential_file}"
EOF
chmod +x "${credential_script}"

%{command}
