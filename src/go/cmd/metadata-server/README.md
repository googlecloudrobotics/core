# Metadata Server

The `metadata-server` runs on the robot cluster and emulates a subset of the GCE/GKE instance metadata server (`http://metadata.google.internal` / `169.254.169.254`). It provides Google Cloud Application Default Credentials (ADC) and connected cloud project metadata to workloads running on the robot so they can seamlessly authenticate with Google Cloud APIs without managing key files directly.

## How it works

1. **Traffic Redirection**: On startup, the server binds to `--bind_ip`:`--port`, adds an `nftables` NAT rule redirecting traffic addressed to `169.254.169.254:80` to the local listener, and patches the cluster's CoreDNS `Corefile` so `metadata.google.internal` resolves properly (both are cleaned up on shutdown).
2. **Token Minting**: When a pod requests `/computeMetadata/v1/instance/service-accounts/default/token` (or `/identity`), `metadata-server` verifies that the peer IP belongs to `--source_cidr`, signs a JWT using the robot's private key (`--robot_id_file`), and exchanges it with the cloud [`token-vendor`](../token-vendor/README.md) for a short-lived GCP OAuth2 access token for `--service_account`.
3. **Project & Instance Metadata**: Serves project details (such as project ID, numeric project number, zone, and robot attributes) under `/computeMetadata/v1/`.

For an overview of the end-to-end authentication flow, see [Device Identity](../../../../docs/concepts/device_identity.md).

## Flags

- `--bind_ip`: IPv4 address to listen on (default: `"127.0.0.1"`).
- `--port`: Port number to listen on (default: `80`).
- `--robot_id_file`: Path to the `robot-id.json` file containing the robot's credentials (default: `""`).
- `--source_cidr`: CIDR specifying allowed source IP addresses for token retrieval (default: `"127.0.0.1/32"`).
- `--min_token_expiry`: Minimum remaining validity time in seconds before a cached token is refreshed (default: `300`).
- `--log_peer_details`: Log details about the pod/peer requesting Application Default Credentials at the expense of extra latency (default: `false`).
- `--log_level`: Minimum log message level required to be logged (default: `0` / `INFO`).
- `--running_on_gke`: Skip CoreDNS patching steps that are unnecessary when running on GKE (default: `false`).
- `--service_account`: Default robot GCP service account name (default: `"robot-service"`).
