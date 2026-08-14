# Envoy Gateway and Gateway API Manifests

This directory contains vendored Custom Resource Definitions (CRDs) and system manifests for Envoy Gateway and Gateway API (standard channel).

- **Envoy Gateway**: [v1.8.3](https://github.com/envoyproxy/gateway/releases/tag/v1.8.3)
- **Gateway API**: [v1.5.1](https://github.com/kubernetes-sigs/gateway-api/releases/tag/v1.5.1)

## Updating Manifests

To update or re-generate the Envoy Gateway and Gateway API manifests, run:

```shell
./update-envoy-gateway.sh
```

You can optionally override the versions by setting environment variables:

```shell
EG_VERSION=v1.8.3 GAPI_VERSION=v1.5.1 ./update-envoy-gateway.sh
```
