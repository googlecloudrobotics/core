# Setup Robot

`setup-robot` bootstraps a robot's local Kubernetes cluster, registers the robot with a Cloud Robotics GCP project, and deploys the core robot platform services.

## How it works

1. **Pre-flight Validation**: Waits for DNS availability, fetches the project configuration from GCP, verifies that the target cluster is not a cloud cluster (by checking for `app-rollout-controller`), and ensures the cluster is not being renamed over an existing installation.
2. **Robot Authentication**: Generates an RSA private/public key pair for the robot (`robot-<robot-name>`), publishes the public key to the cloud [`token-vendor`](../token-vendor/README.md), stores the robot credentials in a local Kubernetes Secret, and initializes GCR image pull secrets.
3. **Core Chart Installation**: Initializes [`synk`](../synk/README.md) on the local cluster and renders/applies the `base-robot` Helm chart (which starts components such as `cr-syncer`, `chart-assignment-controller`, `metadata-server`, `http-relay-client`, and `gcr-credential-refresher`).
4. **Robot Resource Registration**: Creates or updates the `Robot` custom resource in the cloud cluster (or in the local cluster if `--cr-syncer=false`) with the specified `--robot-type`, `--labels`, and `--annotations`.

For an overview of how robot identity and registration work, see [Device Identity](../../../../docs/concepts/device_identity.md).

## Usage

```bash
ACCESS_TOKEN=<gcp-access-token> REGISTRY=<container-registry> \
  setup-robot <robot-name> --project <project-id> [OPTIONS]
```

## Environment Variables

- `ACCESS_TOKEN` (required): OAuth2 access token of a human administrator or service account authorized to read project config and publish keys to the `token-vendor`.
- `REGISTRY` (required): Container image registry prefix used when rendering the `base-robot` Helm chart.
- `HOST_HOSTNAME` (optional): Hostname of the robot machine, recorded in the `cloudrobotics.com/master-host` annotation.
- `CRC_VERSION` (optional): Cloud Robotics Core version string, recorded in the `cloudrobotics.com/crc-version` annotation.

## Flags

- `--project`: Google Cloud Platform Project ID (required).
- `--robot-type`: Robot type identifier (optional if the robot is already registered).
- `--registry-id`: ID used when writing the public key to the cloud registry (default: `"robot-<robot-name>"`).
- `--labels`: Comma-separated `key=value` labels to attach to the `Robot` custom resource.
- `--annotations`: Comma-separated `key=value` annotations to attach to the `Robot` custom resource.
- `--cr-syncer`: Set up `cr-syncer` and create the `Robot` CR in the cloud cluster (default: `true`).
- `--fluentd`: Set up `fluentd` to upload logs to Cloud Logging (default: `true`). Cannot be enabled together with `--fluentbit`.
- `--fluentbit`: Set up `fluentbit` to upload logs to Cloud Logging (default: `false`).
- `--log-prefix-subdomain`: Subdomain to prepend to the Fluentbit log tag prefix (default: `""`).
- `--docker-data-root`: Docker data root directory matching `data-root` in `/etc/docker/daemon.json` (default: `"/var/lib/docker"`).
- `--pod-cidr`: Pod IP address range in the cluster matching the CNI configuration, used by `metadata-server` to allow pod requests (default: `"192.168.9.0/24"`).
- `--robot-authentication`: Generate and register robot authentication credentials (default: `true`).
- `--running-on-gke`: Skip host-specific setup steps that are unnecessary when running on GKE (default: `false`).
