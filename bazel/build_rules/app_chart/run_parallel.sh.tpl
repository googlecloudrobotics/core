#!/usr/bin/env bash

set -eu

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

# Wait for all of the subprocesses, failing the script if any of them failed.
exitcode=0
for pid in "${PIDS[@]}"; do
    wait ${pid} || exitcode=$?
done
exit $exitcode
