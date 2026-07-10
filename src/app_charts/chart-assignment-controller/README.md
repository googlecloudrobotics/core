# chart-assignment-controller

This chart deploys the `chart-assignment-controller`, which applies Helm charts assigned to a robot.

## Building the chart

To build the chart tarball:

```bash
bazel build //src/app_charts/chart-assignment-controller:chart-assignment-controller
```

The packaged chart will be available at `bazel-bin/src/app_charts/chart-assignment-controller/chart-assignment-controller.tgz`.

## Installing the chart

Install the chart using `helm install`. You must specify the `robotName`. If `cert-manager` is not available in the cluster, you must also disable the webhook.

Example installation:

```bash
helm install chart-assignment-controller \
  bazel-bin/src/app_charts/chart-assignment-controller/chart-assignment-controller.tgz \
  --set robotName="my-robot" \
  --set webhook.enabled=false
```
