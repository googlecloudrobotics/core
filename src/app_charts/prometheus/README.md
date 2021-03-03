# Prometheus App

The Prometheus app uses the upstream Helm chart
`prometheus-community/prometheus-operator` as a basis for cloud and robot chart
alike.

The folowing caveats exist that explain some choices made:

* The prometheus-operator chart is vendored in `third_party/` as Helm
  packaging does not generate reproducible artifacts that can be downloaded and
  verified via file checksums at build time.
* Helm cannot template `values.yaml` files themselves. For example, we need to
  provide an external URL for Prometheus, which must include the project domain
  name. But it is only provided at deploy time as `.Values.domain`.
  For this reason, we expand the prometheus-operator chart at build time
  and insert pseudo-variables, which are string-replaced by Helm at deploy time
  during template processing (see `cloud/prometheus-operator.yaml`).
