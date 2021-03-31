# synk

synk is a tool to sync manifests with a cluster.

It takes set of fully populated Kubernetes resources (files or in-process objects) and applies them to a cluster as a named collection. A custom resource is used to store which manifests are part of the set, to reliably cleanup resources that are no longer part of it. Custom Resource Definitions (CRDs) are properly initialized before any other resources are installed and resource dependencies are resolved through retry logic.

It has a similar intent to [Mortar](https://github.com/kontena/mortar), but synk is usable as a Golang library and has first-class support for CRDs.

To be compatible with some of the existing charts, we allow charts to install resources to "kube-system" as the only allowed namespace outside of the chart namespace.

## Examples

```
# Apply my-chart.
helm template my-chart.tgz ... | synk apply my-chart -n default -f -

# Remove my-chart.
synk delete my-chart.v1 -n default
```

## Behavior

synk's `apply` command works as follows:

1. Resources are parsed from the input. Any namespaced resources that don't
   specify a namespace already are updated with the value of the `--namespace`
   (`-n`) flag.
1. Any resources that specify a namespace other than `kube-system` or the given
   namespace will cause synk to fail.
1. synk creates a new `ResourceSet`, listing the resources that are to be
   applied. If reapplying a previously applied set, it creates a new
   ResourceSet with an incremented version number (eg `my-chart.v2`).
1. The resources are split into two groups: CRDs and non-CRDs ("regular
   resources").
1. All CRDs are applied to the cluster. synk then waits for these to become
   available.
1. Next, regular resources are applied to the cluster.

  - Updates: most resources are updated with PATCH requests, which reduces the
    risk of resource version conflicts. Resources larger than 256kB, which can't
    use an annotation to store the last-applied-configuration, are updated with
    POST requests. Resources that can't be updated (eg Jobs, PersistentVolumes)
    are deleted and recreated according to the `canReplace()` heuristic.

  - Ownership: all resources specify the ResourceSet as via ownerReferences.
    This means that the Kubernetes garbage collector will delete the resources
    when the ResourceSet is deleted. Note that CRDs don't have ownerReferences,
    as this presents a data loss risk: if the garbage collector deleted a CRD it
    would also delete all corresponding CRs.

  - Retries: if a transient error is encountered when applying any regular
    resource, synk retries the failed resources until the number of failed
    resources is stable. This retry loop exists to handle constraints of the
    resource creation order: for example, a Pod must be created after the
    ServiceAccount that it uses. This isn't relevant when using Deployments
    instead of bare Pods, so in practice this retry loop is not essential,
    although it could be useful when using CRDs and validation webhooks.

1. The ResourceSet status is updated, describing for each resource whether it
   was successfully applied or not.
1. If all resources were successfully applied, any previous ResourceSets with
   the same name are deleted. This means that if a resource is removed from a
   chart, the Kubernetes garbage collector will delete it after synk
   successfully reapplies the newer version of the chart.
1. Finally, if this process failed due to a transient error (according to the
   IsTransientErr() heuristic) and `--retries=0` hasn't been specified, `apply`
   will be retried completely, including creation of a new ResourceSet. This
   handles transient errors that take seconds or minutes to pass, such as
   apiserver downtime.
