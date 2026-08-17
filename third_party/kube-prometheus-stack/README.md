# Prometheus-operator updates

* [Kube Prometheus Stack](https://github.com/prometheus-community/helm-charts/tree/main/charts/kube-prometheus-stack#kube-prometheus-stack) (go to the tag you consider installing)  
* Values (this also lists the versions of the sub-components, search for tag:): [kube-prometheus-stack/values.yaml](https://github.com/prometheus-community/helm-charts/blob/main/charts/kube-prometheus-stack/values.yaml)  
* Deps (see dependencies section, seems to be minimum versions though): [kube-prometheus-stack/Chart.yaml](https://github.com/prometheus-community/helm-charts/blob/main/charts/kube-prometheus-stack/Chart.yaml)

## Update the chart

Run `./update.sh` to create a commit that update the chart and the CRDs.
Next push the resulting change for review.
