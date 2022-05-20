#!/bin/bash

# match the version to the app version in this command:
# helm search prometheus-community/kube-prometheus-stack --version='x.y.z' --versions
VERSION=0.47
# https://github.com/prometheus-operator/prometheus-operator/tree/main/example/prometheus-operator-crd
# TODO(ensonic): can we tak this directly from the chart?
BASEURL="https://raw.githubusercontent.com/prometheus-operator/prometheus-operator/release-${VERSION}/example/prometheus-operator-crd/monitoring.coreos.com"

OUT=00-crds.yaml
curl > "${OUT}" ${BASEURL}_probes.yaml

OUT=01-crds.yaml
echo '{{ if eq .Values.app_management "true" }}' > "${OUT}"
for CRD in alertmanagerconfigs alertmanagers prometheuses prometheusrules podmonitors servicemonitors thanosrulers; do
  # these already have "---" separators
  curl >> "${OUT}" ${BASEURL}_${CRD}.yaml
done
echo '{{ end }}' >> "${OUT}"
