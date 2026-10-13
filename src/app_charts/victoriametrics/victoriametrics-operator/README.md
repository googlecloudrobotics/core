# VictoriaMetrics Operator

This directory provides the base operator application for the VictoriaMetrics
stack.

## Why a standalone Operator App?

In a typical deployment, the VictoriaMetrics Operator is bundled and deployed
alongside the `victoria-metrics-k8s-stack` chart.

In this architecture, two independent TSDB clusters
(`victoriametrics-cloudmetrics` and `victoriametrics-robotmetrics`) can be
deployed in the same cloud environment. If both applications deployed their own
bundled operators, the two operators would fight to reconcile the same
`VMCluster` and `VMAgent` custom resources, causing Pod flapping.

Extracting the operator into this standalone singleton application gives three
benefits:

1. Only one operator runs in the cluster, so two operators never fight over the
   same custom resources.
2. Setting `watchNamespaces: []` runs the operator cluster-wide. That lets
   `vm-operator` watch `VMCluster`, `VMAgent`, `VMAlert`, and
   `ServiceMonitor`/`PodMonitor` objects across all namespaces, and also makes
   the operator create a `ClusterRole` (instead of a namespace-scoped `Role`)
   for `vmagent` at runtime so `vmagent` can scrape other namespaces like
   `app-nginx` and `app-token-vendor`.
3. The `cloudmetrics` and `robotmetrics` charts only contain custom resources,
   so they can be deployed, updated, or deleted independently without restarting
   the operator.

## Rollout Requirements (`insrc`)

When deploying to an environment in `insrc`:

1. Deploy this `victoriametrics-operator` application before or alongside any
   other `victoriametrics-*` apps.
2. Keep the bundled operators disabled in the `cloudmetrics` and `robotmetrics`
   charts via `victoria-metrics-operator: { enabled: false }` to avoid
   duplicate operator instances.
