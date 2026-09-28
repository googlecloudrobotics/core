# App Rollout Controller

The `app-rollout-controller` runs in the cloud cluster and manages the deployment of applications across cloud and robot clusters. It reconciles `App` and `AppRollout` custom resources by creating, updating, and deleting `ChartAssignment` custom resources to bring cluster deployments into agreement with configuration.

## How it works

1. **Watches Custom Resources**: Monitors `AppRollout`, `App`, `Robot`, and `ChartAssignment` resources in the cloud cluster.
2. **Evaluates Robot Selectors**: Matches `Robot` resources against label selectors defined in each `AppRollout` specification.
3. **Generates `ChartAssignment` Resources**: Creates a `ChartAssignment` for the cloud component and for each matching robot cluster, merging global configuration (`--params`), `App` defaults, and `AppRollout` overrides.
4. **Aggregates Status**: Watches the status of owned `ChartAssignment` resources (which are reconciled by the [`chart-assignment-controller`](../chart-assignment-controller/README.md)) and updates the `AppRollout` status accordingly.
5. **Admission Webhooks**: Serves validating admission webhooks at `/approllout/validate` and `/app/validate` to verify `AppRollout` and `App` manifests upon creation or update.

For more details on the application management architecture, see [App Management](../../../../docs/concepts/app-management.md).

## Flags

- `--params`: Helm configuration parameters formatted as `name=value,topname.subname=value` (default: `""`).
- `--healthz-port`: Listening port of the `/healthz` probe (default: `8080`).
- `--webhook-port`: Listening port of the custom resource validation webhook server (default: `9876`).
- `--cert-dir`: Directory containing TLS certificates for the webhook server (default: `""`).
- `--log-level`: Minimum log message level required to be logged (default: `0` / `INFO`).
