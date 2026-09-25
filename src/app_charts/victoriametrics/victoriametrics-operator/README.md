# VictoriaMetrics Operator

This directory provides the base operator application for the VictoriaMetrics stack.

## Why a standalone Operator App?
In a typical deployment, the VictoriaMetrics Operator is natively bundled and deployed alongside the `victoria-metrics-k8s-stack` chart. 

However, in this architecture, we have multiple independent TSDB clusters (`victoriametrics-cloudmetrics` and `victoriametrics-robotmetrics`) that can be deployed independently in the same cloud environment. If both of those applications deployed their own bundled operators, the two identical operators would constantly fight to reconcile the exact same `VMCluster` and `VMAgent` Custom Resources (CRDs), causing severe cluster instability and Pod flapping.

By extracting the Operator into this standalone, global singleton application:
1. **Zero Collisions**: We guarantee exactly one operator runs globally in the cluster.
2. **Global Scraping RBAC**: This operator is explicitly configured to watch all namespaces (`watchNamespaces: []`). This grants the generated `vmagent` proxies the necessary ClusterRole permissions to dynamically discover and scrape `ServiceMonitor`s located in cross-namespace deployments (like `app-nginx` or `app-token-vendor`).
3. **Total Modularity**: `cloudmetrics` and `robotmetrics` are reduced to simple declarative manifests (containing only the CRs). They can be cleanly deployed, updated, or destroyed completely independently without taking down the orchestration layer.

## Rollout Requirements (`insrc`)
When deploying to an environment in `insrc`:
1. You **must** deploy this `victoriametrics-operator` application before or alongside any other `victoriametrics-*` apps.
2. The `cloudmetrics` and `robotmetrics` charts **must** have their bundled operators disabled via `victoria-metrics-operator: { enabled: false }` to prevent deploying duplicate instances.