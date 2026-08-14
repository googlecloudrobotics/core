# VictoriaMetrics - robot metrics

Collects metrics from robots. Robot-side agents scrape local targets and
write into the cloud VictoriaMetrics cluster; the cloud side runs the
actual VictoriaMetrics query/write/alerting stack.

Separate from `victoriametrics-cloudmetrics`, which is cloud-only - no
robot component, collects cloud service and cloud k8s metrics instead.
