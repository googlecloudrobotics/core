#!/bin/bash
# needs helm3
OUTPUT=./robot/fluent-bit.yaml
helm repo add fluent https://fluent.github.io/helm-charts
helm repo update fluent
helm template fluent-bit fluent/fluent-bit -f fluent-bit-values.yaml --skip-tests > ${OUTPUT}

sed -i '1i\{{ if and (eq .Values.robot_authentication "true") (eq .Values.fluentbit "true") }}' ${OUTPUT}
sed -i '$a\{{ end }}' ${OUTPUT}
sed -i 's/MY_ROBOT/{{ .Values.robot.name }}/' ${OUTPUT}
# This needs to be an actual Cloud zone so that it can be mapped
# to a Monarch/Stackdriver region. TODO(swolter): We should make
# this zone configurable to avoid confusing users.
sed -i 's/MY_CLUSTER_LOCATION/europe-west1-c/' ${OUTPUT}
