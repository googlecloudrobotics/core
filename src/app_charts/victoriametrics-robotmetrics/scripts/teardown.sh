#!/bin/bash
# Removes the VictoriaMetrics AppRollout from a target cluster.
#
# Usage: ./teardown.sh <cluster-context>
#
# Note: per CRC convention (see go/intrinsic-crc-dev), simply deleting
# an AppRollout removes it cleanly via the chart-assignment-controller
# - this is the safe path for a test/personal deployment. The "deploy
# an empty chart first" caveat discussed with Stefan applies when
# REPLACING one AppRollout with another in production (e.g. swapping
# Prometheus for VictoriaMetrics on a live project) to avoid orphaned
# resources - not needed for a straightforward teardown of a test
# deployment like this one.

set -euo pipefail

if [ $# -ne 1 ]; then
  echo "Usage: $0 <cluster-context>"
  exit 1
fi

CONTEXT="$1"

echo "Deleting VictoriaMetrics AppRollout from context ${CONTEXT}..."
kubectl --context="${CONTEXT}" delete -f "$(dirname "$0")/victoriametrics-approllout.yaml" --ignore-not-found

echo "Verifying chart assignments were removed..."
sleep 5
REMAINING=$(kubectl --context="${CONTEXT}" get chartassignments -o name 2>/dev/null | grep -c "victoriametrics" || true)
if [ "${REMAINING}" -eq 0 ]; then
  echo "Teardown complete, no remaining VictoriaMetrics chart assignments."
else
  echo "WARNING: ${REMAINING} VictoriaMetrics chart assignment(s) still present. Check manually:"
  echo "  kubectl --context=${CONTEXT} get chartassignments | grep victoriametrics"
fi
