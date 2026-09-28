# CR Syncer

The `cr-syncer` (Custom Resource Syncer) synchronizes Kubernetes Custom Resources (CRs) between a remote (cloud) Kubernetes cluster and the local (robot) Kubernetes cluster. The `spec` section and resource lifecycle are copied from upstream (cloud) to downstream (robot), while the `status` section is copied from downstream (robot) back to upstream (cloud).

For a detailed explanation of federation semantics, deletion behavior, and resource generations, see [Federation](../../../../docs/concepts/federation.md).

## CRD Annotations

The synchronization behavior is configured per CustomResourceDefinition (CRD) using annotations:

- **`cr-syncer.cloudrobotics.com/spec-source`**: `<string>`
  If set to `"cloud"`, the source of truth for object existence and `spec` (upstream) is the remote cloud cluster, and the source of truth for `status` is the local robot cluster (downstream). If set to `""` or omitted, the CRD is ignored by `cr-syncer`.
- **`cr-syncer.cloudrobotics.com/filter-by-robot-name`**: `<bool>`
  If `true`, only syncs CRs that have a label `cloudrobotics.com/robot-name: <robot-name>` matching the `--robot-name` flag given on the command line.
- **`cr-syncer.cloudrobotics.com/status-subtree`**: `<string>`
  If specified, only syncs the given subtree of the `status` field from downstream to upstream. This is useful when resources share a status across multiple components.

## HTTP Endpoints

The `cr-syncer` listens on `--listen-address` (default `:80`) and serves:

- `/health`: Health check endpoint that verifies connectivity to the remote Kubernetes API server.
- `/metrics`: Prometheus metrics endpoint (including OpenTelemetry HTTP client metrics labeled by `location="local"` and `location="remote"`).
- `/debug`: OpenTelemetry zPages trace debugging handler.

## Flags

- `--remote-server`: Remote Kubernetes server address (required).
- `--robot-name`: Name of the robot `cr-syncer` is running on, used for selective syncing when `filter-by-robot-name` is enabled.
- `--listen-address`: HTTP listen address for health, metrics, and debug endpoints (default: `":80"`).
- `--conflict-error-limit`: Number of consecutive conflict errors before an informer is restarted (default: `5`).
- `--timeout`: Timeout for CR watch calls in seconds (default: `300`).
- `--use-robot-jwt`: Use a robot-signed JWT for authentication instead of a GCP OAuth2 access token (default: `false`).
- `--log-level`: Minimum log message level required to be logged (default: `0` / `INFO`).
- `--verbose`: Deprecated; enables debug logging and HTTP request/response logging (default: `false`).
