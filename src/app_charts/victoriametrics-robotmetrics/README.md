# VictoriaMetrics Robot Metrics

This app collects and stores metrics originating from robots. It has both robot
and cloud components: robot-side collectors scrape robot-local targets and write
to the cloud-side VictoriaMetrics cluster, while the cloud component provides the
VictoriaMetrics query/write services and alerting for that robot metrics stream.

This is distinct from `victoriametrics-cloudmetrics`, which collects metrics from
cloud services and cloud-cluster Kubernetes infrastructure only. That app is
cloud-only and does not deploy robot-side collectors.
