Helm chart for akri (version 0.8.23) included here was pulled from the project-akri github repo using the following steps and modified to be compatible with the Intrinsic codebase:
```
helm repo add akri-helm-charts https://project-akri.github.io/akri/ 
helm pull akri-helm-charts/akri
tar -xvf <downloaded-tgz-file-name>
```
