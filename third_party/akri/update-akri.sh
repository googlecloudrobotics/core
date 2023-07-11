#!/bin/bash

VERSION=0.8.23

# fetch the latest version
helm repo add akri-helm-charts https://project-akri.github.io/akri/ 
helm repo update akri-helm-charts
helm pull akri-helm-charts/akri --version="${VERSION}"

# Update crds
tar xzf "akri-${VERSION}.tgz" --strip-components=2 akri/crds/
# Strip template comments
sed -i 's/#.*$//' akri-configuration-crd.yaml
