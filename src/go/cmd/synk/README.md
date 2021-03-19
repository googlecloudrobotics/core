# synk

synk is a tool to sync manifests with a cluster.

It takes set of fully populated Kubernetes resources (files or in-process objects) and applies them to a cluster as a named collection. A custom resource is used to store which manifests are part of the set, to reliably cleanup resources that are no longer part of it. Custom Resource Definitions (CRDs) are properly initialized before any other resources are installed and resource dependencies are resolved through retry logic.

It has a similar intent to [Mortar](https://github.com/kontena/mortar), but synk is usable as a Golang library and has first-class support for CRDs.

To be compatible with some of the existing charts, we allow charts to install resources to "kube-system" as the only allowed namespace outside of the chart namespace.

## Examples

```
# Apply my-chart.
helm template my-chart.tgz ... | synk apply my-resource-set -n default -f -

# Remove my-chart.
synk delete my-chart.v1 -n default
```

