#!/usr/bin/env bash

set -eu

if [[ "$#" -lt 1 ]]; then
  echo "Usage: $0 <container-registry>"
  exit 1
fi
CONTAINER_REGISTRY="$1"

function guess_runfiles() {
    pushd ${BASH_SOURCE[0]}.runfiles > /dev/null 2>&1
    pwd
    popd > /dev/null 2>&1
}

RUNFILES="${PYTHON_RUNFILES:-$(guess_runfiles)}"

PIDS=()
function async() {
    # Launch the command asynchronously and track its process id.
    PYTHON_RUNFILES=${RUNFILES} "$@" &
    PIDS+=($!)
}

%{commands}

if [[ "${#PIDS[@]}" = 0 ]]; then
    # It is valid to generate this script without pushing any images.
    # Bash before v4.4 considers an empty array an unbound variable and would
    # choke on the for-loop below.
    exit 0
fi

# Wait for all of the subprocesses, failing the script if any of them failed.
exitcode=0
for pid in "${PIDS[@]}"; do
    wait ${pid} || exitcode=$?
done
exit $exitcode
