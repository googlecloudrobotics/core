Helm chart for akri (version 0.8.23) included here was pulled from the project-akri github repo using the following steps and modified to be compatible with the Intrinsic codebase:
```
helm repo add akri-helm-charts https://project-akri.github.io/akri/ 
helm pull akri-helm-charts/akri
tar -xvf <downloaded-tgz-file-name>
```

Following modifications were done to the "vanilla" helm chart pulled from the
github repo::
- In `akri/templates/agent.yaml`, `akri/templates/controller.yaml`, `akri/templates/udev-discovery-handler.yaml`, `akri/templates/onvif-discovery-handler.yaml`, and `akri/templates/opcua-discovery-handler.yaml`, changed `.Values.AppVersion` to `.Values.akriAppVersion`, which must be specified in the Intrinsic `<app-name>/values.yaml` file.
- In `akri/template/udev-configuration.yaml`, replaced `name` field under `spec/brokerSpec/brokerPodSpec/containers` with the following lines:
```
- name: {{ .Values.udev.configuration.name }}-broker
    {{ if .Values.udev.configuration.brokerPod.image.custom_image}}
    image: {{ .Values.udev.configuration.brokerPod.image.repository }} {{- .Values.images.simple_broker }}
    {{ else }}
    image: {{ printf "%s:%s" .Values.udev.configuration.brokerPod.image.repository .Values.udev.configuration.brokerPod.image.tag | quote }}
    {{ end }}
    tty: true
    stdin: true
    {{- if .Values.udev.configuration.brokerPod.args }}
    args: {{ .Values.udev.configuration.brokerPod.args }}
    {{- end }}
```

