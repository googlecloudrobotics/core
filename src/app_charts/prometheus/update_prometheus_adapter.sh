#!/bin/bash

VERSION="0.12.0"
OUT="robot/prometheus-adapter.yaml"
wget https://github.com/kubernetes-sigs/prometheus-adapter/archive/refs/tags/v"${VERSION}".tar.gz
tar xvzf v"${VERSION}".tar.gz

awk 'FNR==1 && NR>1 {print "---"}{print}' "prometheus-adapter-${VERSION}/deploy/manifests/"*.yaml > "${OUT}"
sed -i 's#replicas: 2#replicas: 1#g' "${OUT}"
sed -i 's#namespace: monitoring#namespace: {{ .Release.Namespace }}#g' "${OUT}"
sed -i 's#https://prometheus.monitoring.svc#http://kube-prometheus.{{ .Release.Namespace }}.svc#g' "${OUT}"

