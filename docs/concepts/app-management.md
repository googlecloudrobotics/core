# App Management

The Cloud Robotics Core application management (Layer 2) makes it easy to define and deploy
arbitrary applications across a fleet of cloud and robot clusters. The
[Helm v2 chart format](https://helm.sh/docs/developing_charts/) is used to define an application
at the scope of a single cluster. Additional custom resources tie them together to a
cross-cluster application and define their deployment. It relies on the Cloud Robotics Core
[federation layer](federation.md) to distribute the resources to the right clusters.

## App resource

The App resource defines a Cloud Robotics Core application by simply describing Helm charts for
two classes of clusters: cloud and robots. Charts may be specified inline as base64-encoded Helm
chart files or by referencing a Helm repository. App resources will be deployed to the cloud
cluster.

Example 1: application referencing charts from helm repositories

```yaml
apiVersion: apps.cloudrobotics.com/v1alpha1
kind: App
metadata:
  name: ros-v1
spec:
  repository: https://my.repo
  version: 1.2.1
  components:
    cloud:
      name: ros-cloud
    robot:
      name: ros-robot
```

Example 2: application using inline as base64-encoded charts (development workflow)

```yaml
apiVersion: apps.cloudrobotics.com/v1alpha1
kind: App
metadata:
  name: ros-v1
spec:
  components:
    cloud:
      chart:
        inline: <base64>
    robot:
      chart:
        inline: <base64>
```

Right now we only have bazel build rules to produce inline charts.

## AppRollout Resource

An AppRollout describes how a defined App should be deployed across a fleet of clusters. It allows
to flexibly select robots which should run an application and to inject fine-grained configuration.

Example 3: AppRollout with different configuration options per target

```yaml
apiVersion: apps.cloudrobotics.com/v1alpha1
kind: AppRollout
metadata:
  name: ros-stable
  labels:
    app.kubernetes.io/name: ros
    role: navtest
    release: stable
spec:
  appName: ros-v1
  cloud:
    values:
      override: foo
  robots:
  - selector:
      matchLabels:
        model: mir100
    values:
      override1: bar
  - selector:
      matchLabels:
        model: mir200
    values:
      override1: baz
    version: v1.2.2   # Chart version override for canarying
```

AppRollouts are deployed into the cloud cluster, where a controller (app-rollout-controller) handles them.
The controller applies the specified selectors and creates or updates the internal
ChartAssignments. A ChartAssignment represents a single instance of a chart that should be
installed into a single cluster. These internal objects describe the task of installing parts of
the application.

The federation layer will sync ChartAssignments to robots as needed. The actual
installation is done by another controller (chart-assignment-controller), this
time running both in the cloud and on the robots. The AppRollout controller will
watch the status updates and consolidate the information into status updates on
the AppRollout.

## Sharing secrets

If you create a Secret in the `default` namespace labelled
`cloudrobotics.com/copy-to-chart-namespaces=true`, it will be copied into all
namespaces created by the chart-assignment-controller. This is useful for
cluster-specific license keys that can be used by applications.

## Opt a pod out of status checking

During rollout, the chart-assignment-controller checks for Pods in the rollout being `Running` or `Completed`.
In some cases, this check is not necessary or might need to be opted out of.
In this case, add a label `cloudrobotics.com/opt-out-error-checking=true` to your pods. Adding this
instructs the chart-assignment-controller to not block the status from reaching `Ready`.

## Development workflows: overriding resources

During development, you might want to temporarily modify a resource (e.g. edit a
Deployment's arguments or environment variables) that is managed by an `AppRollout`
and `ChartAssignment`. By default, the `chart-assignment-controller` will detect
these manual changes as drift and overwrite them to match the chart definition.

To temporarily bypass this and prevent the controller from overwriting your changes:

1. Edit the resource in the cluster (e.g., using `kubectl edit`).
2. Add the annotation `synk.cloudrobotics.com/ignore: "true"` to the resource's metadata.
3. Make your desired changes and save.

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: my-deployment
  annotations:
    synk.cloudrobotics.com/ignore: "true"
spec:
  ...
```

When this annotation is present, the reconciliation library (`synk`) will skip
updating the resource spec, but will still maintain its owner references so it
does not get garbage collected. The resource will be reported with the `Ignored`
action in the `ResourceSet` status.

To revert to the state defined in the chart, simply remove the
`synk.cloudrobotics.com/ignore` annotation (or set it to `"false"`). The next
reconciliation will restore the original configuration.
