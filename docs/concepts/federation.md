# Federation

Federation, part of Layer 1, is responsible for synchronizing the state between robot and cloud
clusters. Configuration state in Cloud Robotics Core is primarily expressed through [custom
resources](https://kubernetes.io/docs/concepts/extend-kubernetes/api-extension/custom-resources/)
by platform and user applications alike. Our federation system enables other components to use
custom resources locally without needing to be aware of the multi cluster setup and the quality
of the network connection.

## Semantics
A Kubernetes resource is typically divided into a [“spec” and a “status”](https://kubernetes.io/docs/concepts/overview/working-with-objects/kubernetes-objects/#object-spec-and-status)
section. The `spec` section expresses the intent of the resource, typically authored by a user
or another application along with all its metadata.
The `status` section must generally only be written to by the controller that is responsible for
realizing the specification. Consequently, `spec` and `status` typically each have one distinct
author.

For federating resources, this means that `spec` and `status` of a resource are owned by at most
one cluster respectively (possibly the same one). The cluster owning the spec is also the main
owner of the resource overall and controls its lifecycle, i.e. deletion.
A resource’s spec is always synced from an upstream cluster to a downstream
cluster and its status synced back from downstream to upstream.

All resources of a specific type may either be synchronized to all robots or to exactly one robot.
There is no direct synchronization between robots. However, a robot may create a resource in the
cloud cluster that will be distributed to other robots.

If a resource owned by the upstream cluster has been synchronized to one or more
downstream clusters, it can only be permanently deleted upstream: if deleted
downstream, it will be recreated. If deleted in the upstream cluster, it will be
asynchronously deleted in other clusters that hold a copy of the resource.
Upstream deletion can complete before the downstream resource is deleted.

## cr-syncer
The cr-syncer component (Custom Resource Syncer) is a controller that runs inside each robot
cluster. It is connected to the Kubernetes API servers of the cloud and the robot cluster alike
and continuously watches for updates on custom resources. The controller contains retry and resync
logic to address intermittent connectivity.

![federation](federation.png)

The behavior of the cr-syncer can be configured per custom resource definition (CRD) by setting
annotations on its CRD:

* `cr-syncer.cloudrobotics.com/spec-source`: may be `cloud` or `robot`. It determines which
  cluster type owns metadata, spec, and lifecycle of all resources of the CRD. It implies
  that the other cluster type owns the status section.
* `cr-syncer.cloudrobotics.com/filter-by-robot-name`: a boolean that determines whether resources
  will be synced to all robots or just a single one. An individual resource is labeled with
  `cloudrobotics.com/robot-name` to indicate which robot it should be synced to. If the label
  is missing on a resource, it will not be synced at all.
* `cr-syncer.cloudrobotics.com/status-subtree`: a string key, which defines which sub-section of
  the resource status is synced from the downstream cluster. This lets you split
  a resource’s status into `robot` and `cloud` sections, for example. Using this
  annotation is generally discouraged as it likely points to a flaw in the
  modeling of the respective CRD.

## Deletion
When the cr-syncer sees a resource in the downstream cluster with no
corresponding resource in upstream cluster, it deletes it. This handles orphaned
resources when the upstream resource was deleted while the cr-syncer was
restarting. It also means that you can't create a resource directly in the
downstream cluster. The upstream resource is identified using the namespace and
name, but not the UID, so deletion and recreation upstream may result in an
update in the downstream cluster.

In some cases, downstream deletion may be blocked. For example, if we have
deleted an upstream ChartAssignment, but the robot-master has failed to remove
its finalizer from the downstream ChartAssignment. This edge case leads to
surprising behavior:

- An upstream ChartAssignment can be recreated before the downstream
  ChartAssignment is deleted.
- The old status from the downstream cluster will be synced to the new upstream
  ChartAssignment.

If needed, this can be detected by watching the downstream cluster after
deleting the resource from the downstream cluster. The situation will clean up
once downstream deletion is complete.

Note: previously, the cr-syncer used finalizers to block upstream deletion
until the downstream resource was deleted. This gave the original deleter more
information: for example, once an AppRollout had been deleted in the cloud, it
means that all robots have terminated the app's pods. However, this caused
problems with offline or renamed clusters: an admin would have to manually clean
up the old finalizers. The new asynchronous behavior is not affected by offline
clusters.

## Resource generations

Custom resources have a field `.metadata.generation` that starts at 1 and is
incremented when the resource changes. Specifically, if the CRD enables the
/status subresource, the generation increases by 1 every time the resource spec
changes, but not when the status changes. The resource controller can set
`.status.observedGeneration` to the latest generation it has observed, so the
user can change the spec, then wait for `observedGeneration` to catch up before
looking at the status. For example:

* Create a Deployment for one pod (generation=1), and wait for the status to be Ready.
* Change the Deployment's image reference (generation=2): the status is still
  Ready, but this refers the old spec (observedGeneration=1).
* Wait for the status to update (observedGeneration=2): now the status is
  non-ready, referring to the newer spec.
* Wait for the status to be Ready. The new image is now running.

`generation` and `observedGeneration` can **only be compared in the downstream
cluster**. As the generation is managed by the Kubernetes apiserver, the
cr-syncer cannot guarantee that the upstream generation matches the downstream
generation. On the other hand, `observedGeneration` will be copied from
downstream to upstream with the rest of `.status`. This means that `generation`
is cluster-specific but `observedGeneration` always refers to the downstream
generation.
