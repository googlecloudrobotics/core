# Envoy Gateway and Gateway API Manifests

This directory contains vendored Custom Resource Definitions (CRDs) and system manifests for Envoy Gateway and Gateway API (standard channel).

- **Envoy Gateway**: [v1.10.0](https://github.com/envoyproxy/gateway/releases/tag/v1.10.0)
- **Gateway API**: [v1.6.1](https://github.com/kubernetes-sigs/gateway-api/releases/tag/v1.6.1)

## Updating Manifests

Vendored manifests are updated automatically by Renovate. To update or re-generate them manually, run:

```shell
./update.sh
```

You can optionally override the versions by setting environment variables (`GAPI_VERSION` is resolved automatically from Envoy Gateway's `go.mod` when omitted):

```shell
EG_VERSION=v1.10.0 GAPI_VERSION=v1.6.1 ./update.sh
```
