#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
REPO_ROOT=$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)
cd "${SCRIPT_DIR}"

CHART_NAME="kube-prometheus-stack"
REPO_NAME="prometheus-community"

echo "Updating Helm repositories..."
helm repo add "${REPO_NAME}" https://prometheus-community.github.io/helm-charts
helm repo update "${REPO_NAME}"

# Determine current chart version
PROM_OLD=$(ls ${CHART_NAME}-*.tgz 2>/dev/null | head -n 1 | sed -E "s/${CHART_NAME}-(.*)\.tgz/\1/" || true)
echo "Current ${CHART_NAME} version: ${PROM_OLD:-none}"

# Determine latest available chart and app version
PROM_NEW=$(helm search repo "${REPO_NAME}/${CHART_NAME}" -o json | python3 -c "import sys, json; print(json.load(sys.stdin)[0]['version'])")
APP_VER=$(helm search repo "${REPO_NAME}/${CHART_NAME}" -o json | python3 -c "import sys, json; print(json.load(sys.stdin)[0]['app_version'])")

if [[ -z "${PROM_NEW}" ]]; then
  echo "Error: Failed to determine latest version of ${CHART_NAME}." >&2
  exit 1
fi

echo "Latest ${CHART_NAME} version: ${PROM_NEW} (appVersion: ${APP_VER})"

if [[ "${PROM_OLD}" == "${PROM_NEW}" ]]; then
  echo "${CHART_NAME} is already up to date (${PROM_NEW})."
  exit 0
fi

echo "Fetching ${REPO_NAME}/${CHART_NAME} version ${PROM_NEW}..."
helm fetch "${REPO_NAME}/${CHART_NAME}" --version="${PROM_NEW}"

# Update git tracking and references across the repository if replacing an older chart
if [[ -f "${CHART_NAME}-${PROM_NEW}.tgz" ]]; then
  git add "${CHART_NAME}-${PROM_NEW}.tgz"
fi

if [[ -n "${PROM_OLD}" && "${PROM_OLD}" != "${PROM_NEW}" && -f "${CHART_NAME}-${PROM_OLD}.tgz" ]]; then
  git rm "${CHART_NAME}-${PROM_OLD}.tgz"

  echo "Updating references from ${PROM_OLD} to ${PROM_NEW} across repository..."
  FILES_TO_UPDATE=$(git grep -l "${CHART_NAME}-${PROM_OLD}.tgz" "${REPO_ROOT}" 2>/dev/null || true)
  if [[ -n "${FILES_TO_UPDATE}" ]]; then
    echo "${FILES_TO_UPDATE}" | xargs -r sed -i "s/${CHART_NAME}-${PROM_OLD}\.tgz/${CHART_NAME}-${PROM_NEW}\.tgz/g"
    echo "${FILES_TO_UPDATE}" | xargs -r git add
  fi
fi

# Update CRDs using appVersion (strip leading 'v' and keep X.Y for release branch)
if [[ -n "${APP_VER}" ]]; then
  CRD_VER=$(echo "${APP_VER#v}" | cut -d. -f1,2)
  echo "Updating CRDs to Prometheus Operator release-${CRD_VER}..."
  BASEURL="https://raw.githubusercontent.com/prometheus-operator/prometheus-operator/release-${CRD_VER}/example/prometheus-operator-crd/monitoring.coreos.com"

  OUT="00-crds.yaml"
  curl -fsSL "${BASEURL}_probes.yaml" > "${OUT}"
  git add "${OUT}"

  OUT="01-crds.yaml"
  echo '{{ if eq .Values.app_management "true" }}' > "${OUT}"
  for CRD in alertmanagerconfigs alertmanagers prometheuses prometheusrules podmonitors scrapeconfigs servicemonitors thanosrulers; do
    curl -fsSL "${BASEURL}_${CRD}.yaml" >> "${OUT}"
  done
  echo '{{ end }}' >> "${OUT}"
  git add "${OUT}"
fi

if ! git diff --cached --quiet 2>/dev/null; then
  git commit -m "Update ${CHART_NAME} from ${PROM_OLD:-unknown} to ${PROM_NEW} (appVersion ${APP_VER})"
fi

echo "Update complete."
