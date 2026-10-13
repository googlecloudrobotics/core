# VictoriaMetrics Kubernetes Stack (`victoria-metrics-k8s-stack`)

This directory contains the vendored
[`victoria-metrics-k8s-stack`](https://github.com/VictoriaMetrics/helm-charts/tree/master/charts/victoria-metrics-k8s-stack)
Helm chart archive and extracted Custom Resource Definitions (CRDs) from
`https://victoriametrics.github.io/helm-charts`.

* Chart repository: `https://victoriametrics.github.io/helm-charts`
* Default values:
  [`victoria-metrics-k8s-stack/values.yaml`](https://github.com/VictoriaMetrics/helm-charts/blob/master/charts/victoria-metrics-k8s-stack/values.yaml)
* Chart dependencies:
  [`victoria-metrics-k8s-stack/Chart.yaml`](https://github.com/VictoriaMetrics/helm-charts/blob/master/charts/victoria-metrics-k8s-stack/Chart.yaml)

## Upgrading the Chart and CRDs

To upgrade `victoria-metrics-k8s-stack` to a new `<NEW_VERSION>`:

1. Pull the new chart archive in this directory and remove the old `.tgz`:

   ```bash
   cd third_party/victoria-metrics-k8s-stack
   helm repo add vm https://victoriametrics.github.io/helm-charts
   helm repo update vm
   helm pull vm/victoria-metrics-k8s-stack --version "<NEW_VERSION>"
   git rm victoria-metrics-k8s-stack-<OLD_VERSION>.tgz
   ```

2. Extract `charts/victoria-metrics-operator/crd.yaml` (which uses
   `x-kubernetes-preserve-unknown-fields: true` on `spec` to stay within
   `ChartAssignment` size limits) into `00-crds.yaml`:

   ```bash
   tar -zxvf "victoria-metrics-k8s-stack-<NEW_VERSION>.tgz" \
     -O victoria-metrics-k8s-stack/charts/victoria-metrics-operator/crd.yaml \
     > 00-crds.yaml
   ```

3. Update the `.tgz` version in `BUILD.bazel` and the VictoriaMetrics app
   charts:

   ```bash
   OLD_TGZ="victoria-metrics-k8s-stack-<OLD_VERSION>\.tgz"
   NEW_TGZ="victoria-metrics-k8s-stack-<NEW_VERSION>.tgz"
   VM_DIR="../../src/app_charts/victoriametrics"
   sed -i "s/${OLD_TGZ}/${NEW_TGZ}/g" \
     BUILD.bazel \
     "${VM_DIR}/victoriametrics-operator/BUILD.bazel" \
     "${VM_DIR}/victoriametrics-cloudmetrics/BUILD.bazel" \
     "${VM_DIR}/victoriametrics-robotmetrics/BUILD.bazel"
   ```

4. Verify that the pre-rendered charts build cleanly:

   ```bash
   bazel build //src/app_charts/victoriametrics/...
   ```
