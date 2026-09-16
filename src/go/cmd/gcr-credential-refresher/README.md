# GCR Credential Refresher

The `gcr-credential-refresher` runs on the robot cluster and periodically updates the Kubernetes `imagePullSecrets` used to pull container images from Google Container Registry (GCR) and Artifact Registry.

## How it works

1. On startup, and every 10 minutes thereafter, `gcr-credential-refresher` loads the robot's identity and private key from the `robot-id.json` file (`--robot_id_file`).
2. It exchanges a signed JWT with the [`token-vendor`](../token-vendor/README.md) in the cloud cluster to obtain an OAuth2 access token for the robot's GCP service account (`--service_account`).
3. It updates the Docker registry secrets (`gcr-json-key`) and attaches them to the `default` service accounts across namespaces in the local Kubernetes cluster so pods can pull private images.

For background on how robot authentication works, see [Device Identity](../../../../docs/concepts/device_identity.md).

## Flags

- `--robot_id_file`: Path to the `robot-id.json` file containing the robot's identity and private key.
- `--service_account`: Robot default GCP service account name (default: `"robot-service"`).
