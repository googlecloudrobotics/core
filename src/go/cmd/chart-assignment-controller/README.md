# Chart Assignment Controller

The `chart-assignment-controller` ensures that applications assigned to a Kubernetes cluster via `ChartAssignment` custom resources are installed, updated, and removed as configured. It runs in both the cloud cluster and on each robot cluster.

## How it works

1. **Cluster Filtering**: When running in the cloud (`--cloud-cluster=true`), it reconciles `ChartAssignment` resources targeted at the `"cloud"` cluster. When running on a robot (`--cloud-cluster=false`), it reads the `ROBOT_NAME` environment variable and reconciles `ChartAssignment` resources targeted at that robot.
2. **Namespace & Environment Preparation**: For each assigned chart, it ensures the target namespace (`spec.namespaceName`) exists, waits for the default ServiceAccount, configures GCR image pull credentials, and copies any Secrets from the `default` namespace labeled `cloudrobotics.com/copy-to-chart-namespaces=true`.
3. **Chart Rendering & Application**: Renders the inline Helm v2 chart with the specified values and applies the resulting Kubernetes manifests to the local cluster as a `ResourceSet` using [`synk`](../synk/README.md).
4. **Status Reporting**: Watches pods and resources in the chart's namespace to update the `ChartAssignment` status (e.g. `Ready`, `Settled`, or `Failed`), which is then synced back to the cloud cluster and aggregated by the [`app-rollout-controller`](../app-rollout-controller/README.md).
5. **Admission Webhook**: When `--webhook-enabled=true`, serves a validating admission webhook at `/chartassignment/validate`.

For more details on `App`, `AppRollout`, and `ChartAssignment`, see [App Management](../../../../docs/concepts/app-management.md).

## Flags

- `--cloud-cluster`: Whether the controller is deployed in the cloud cluster (default: `true`). If `false`, the `ROBOT_NAME` environment variable must be set.
- `--healthz-port`: Listening port of the `/healthz` probe (default: `8080`).
- `--webhook-enabled`: Whether the validating webhook server should be served (default: `true`).
- `--webhook-port`: Listening port of the custom resource webhook server (default: `9876`).
- `--cert-dir`: Directory for TLS certificates (default: `""`).
- `--trace-stackdriver-project-id`: If non-empty and running on a robot cluster, uploads OpenTelemetry traces to this Google Cloud Project (default: `""`).
- `--apiserver-max-qps`: Maximum number of calls to the Kubernetes API server per second (default: `50`).
- `--log-level`: Minimum log message level required to be logged (default: `0` / `INFO`).
