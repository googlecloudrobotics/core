#!/bin/bash

# fetch the latest version
helm repo add akri-helm-charts https://project-akri.github.io/akri/ 
helm repo update akri-helm-charts
helm pull akri-helm-charts/akri --untar

# Update crds
cp -r ./akri/crds ./crds

# Edit helm-charts
sed -i 's/.Chart.AppVersion/.Values.akriAppVersion/g' ./akri/templates/agent.yaml
sed -i '28,92s/^/  /' akri/templates/agent.yaml
sed -i '98,120s/^/  /' akri/templates/agent.yaml
sed -i '122,124s/^/  /' akri/templates/agent.yaml

sed -i 's/.Chart.AppVersion/.Values.akriAppVersion/g' ./akri/templates/controller.yaml
sed -i '20,48s/^/  /' akri/templates/controller.yaml

# udev
sed -i 's/.Chart.AppVersion/.Values.akriAppVersion/g' ./akri/templates/udev-discovery-handler.yaml
sed -i '/- name: {{ .Values.udev.configuration.name }}-broker/r '<(cat<<'EOF'
        {{ if .Values.udev.configuration.brokerPod.image.custom_image}}
        image: {{ .Values.udev.configuration.brokerPod.image.repository }} {{- .Values.images.simple_broker }}
        {{ else }}
EOF
) akri/templates/udev-configuration.yaml

sed -i '/image: {{ printf \"%s:%s\" .Values.udev.configuration.brokerPod.image.repository .Values.udev.configuration.brokerPod.image.tag | quote }}/r '<(cat<<'EOF'
        {{ end }}
        tty: true
        stdin: true
        {{- if .Values.udev.configuration.brokerPod.args }}
        args: {{ .Values.udev.configuration.brokerPod.args }}
        {{- end }}
EOF
) akri/templates/udev-configuration.yaml

# onvif -May reuire more updates
sed -i 's/.Chart.AppVersion/.Values.akriAppVersion/g' ./akri/templates/onvif-discovery-handler.yaml

# opcua - May require more updates
sed -i 's/.Chart.AppVersion/.Values.akriAppVersion/g' ./akri/templates/opcua-discovery-handler.yaml

# Repackage updated helm-charts 
helm package ./akri

# Cleanup
rm -rf ./akri

