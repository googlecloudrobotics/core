# 3rd party deps

This directory contains bazel support for 3rd-party deps and vendored
dependencies.

## 3rd party helm charts

We store version pinned 3rd party helm charts here to achieve a hermetic build.

To update a chart, first setup helm:

```shell
curl -s https://get.helm.sh/helm-v2.17.0-linux-amd64.tar.gz | tar xzf - -C ~/bin --strip-components=1 linux-amd64/helm
helm init --client-only
```

Next check available versions and fetch the specific version to store:

```shell
helm search prometheus-community/prometheus-operator --versions
helm fetch prometheus-community/prometheus-operator --version=6.12.0
```

