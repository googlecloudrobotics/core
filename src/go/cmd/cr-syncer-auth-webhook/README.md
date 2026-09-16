# CR Syncer Auth Webhook

The `cr-syncer-auth-webhook` runs in the cloud cluster and verifies that incoming Kubernetes API requests from [`cr-syncer`](../cr-syncer/README.md) instances on robots are authenticated and restricted to the resources allowed for the robot named in the request's credentials.

## How it works

1. **JWT Verification**: Extracts the robot's bearer token from the `Authorization` header and delegates signature verification to the [`token-vendor`](../token-vendor/README.md) (`/apis/core.token-vendor/v1/jwt.verify`) to confirm the token was signed by a registered robot's private key.
2. **Robot Identity Extraction**: Parses the verified JWT's `sub` claim to determine the authenticated robot's name (`robot-<name>`).
3. **Request Path Authorization**: Parses the target Kubernetes API path and ensures the robot is not impersonating another robot:
   - Unfiltered cluster-wide resources (such as `registry.cloudrobotics.com/robottypes`) are allowed for all authenticated robots.
   - Filtered resources must either target a `cloudrobotics.com/robot-name=<robot-name>` label selector or a resource name suffixed with the authenticated robot's name.
4. **Credential Exchange & Forwarding**: Replaces the robot's `Authorization` header with the local Kubernetes ServiceAccount token (whose RBAC permissions are restricted to syncable custom resources) so the cloud Kubernetes API server can serve the request.

## HTTP Endpoints

- `/apis/core.kubernetes/`: Reverse-proxies validated `cr-syncer` requests directly to the cloud Kubernetes API server (`--k8s-target`), streaming responses (including long-lived `watch` streams) back to the robot.
- `/auth`: External authentication subrequest webhook handler (e.g. for nginx `auth_request`) that validates the request and returns a `200 OK` with the local ServiceAccount `Authorization` header.
- `/healthz`: Liveness and readiness probe endpoint.
- `/metrics`: Prometheus metrics endpoint.

## Flags

- `--port`: Listening port for HTTP requests (default: `8080`).
- `--accept-legacy-service-account-credentials`: Whether to accept legacy GCP service account access tokens (`ya29.*`) in addition to robot JWTs (default: `false`).
- `--token-vendor`: Base URL of the `token-vendor` service (default: `"http://token-vendor.app-token-vendor.svc.cluster.local"`).
- `--k8s-target`: Target URL for the Kubernetes API server reverse proxy (default: `"https://kubernetes.default.svc:443"`).
- `--k8s-token-path`: Path to the local Kubernetes ServiceAccount token (default: `"/var/run/secrets/kubernetes.io/serviceaccount/token"`).
- `--k8s-ca-path`: Path to the Kubernetes CA certificate (default: `"/var/run/secrets/kubernetes.io/serviceaccount/ca.crt"`).
- `--log-level`: Minimum log message level required to be logged (default: `0` / `INFO`).
