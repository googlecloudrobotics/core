# Setup Dev

`setup-dev` is a command-line utility that configures a developer workstation to interact with a Cloud Robotics GCP project and its registered robots.

## How it works

1. **Reads Cloud Project Configuration**: Connects to the specified GCP project (`--project`) using Application Default Credentials to fetch the Cloud Robotics configuration and domain.
2. **Selects Target Robot**: Lists `Robot` custom resources in the cloud cluster and prompts the user to choose one interactively (unless `--robot-name` is specified).
3. **Configures `kubectl` Context**: Creates or updates a cluster and context entry (`<project>-robot`) in `~/.kube/config` pointing to the cloud `kubernetes-relay` endpoint (`https://<domain>/apis/core.kubernetes-relay/client/<robot-name>`), allowing `kubectl --context <project>-robot` to run commands against the remote robot cluster.
4. **Registers Workstation Credentials**: Generates an RSA key pair for the workstation (`dev-<hostname>`), publishes the public key to the cloud [`token-vendor`](../token-vendor/README.md), and stores the private key locally for authenticated tools (such as SSH).

## Flags

- `--project`: Google Cloud Platform Project ID (required).
- `--robot-name`: Target robot name (optional; if omitted, prompts interactively).
