#!/bin/bash
# Deploys the VictoriaMetrics AppRollout to a target CRC-managed
# project's cloud-robotics cluster for testing.
#
# Usage: ./setup.sh <cluster-context>
#
# Prerequisites:
#   - kubectl configured with credentials for the target cluster
#     (e.g. via `gcloud container clusters get-credentials`)
#   - The victoriametrics app chart already released to the target
#     cluster's CRC install (see releasing instructions in the CRC
#     dev workflow doc)

set -euo pipefail

if [ $# -ne 1 ]; then
  echo "Usage: $0 <cluster-context>"
  exit 1
fi

CONTEXT="$1"

echo "Applying VictoriaMetrics AppRollout to context ${CONTEXT}..."
kubectl --context="${CONTEXT}" apply -f "$(dirname "$0")/victoriametrics-approllout.yaml"

echo "Waiting for chart assignments to become Ready..."
for _ in $(seq 1 30); do
  STATUS=$(kubectl --context="${CONTEXT}" get chartassignments -o json 2>/dev/null \
    | python3 -c "import json,sys; d=json.load(sys.stdin); print(all(i['status'].get('phase')=='Ready' for i in d['items'] if 'victoriametrics' in i['metadata']['name']))" 2>/dev/null || echo "False")
  if [ "${STATUS}" = "True" ]; then
    echo "All VictoriaMetrics chart assignments Ready."
    exit 0
  fi
  sleep 10
done

echo "WARNING: chart assignments did not reach Ready within 5 minutes. Check manually:"
echo "  kubectl --context=${CONTEXT} get chartassignments | grep victoriametrics"
exit 1
