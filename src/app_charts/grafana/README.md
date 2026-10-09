# Grafana App

The Grafana app uses the upstream `kube-prometheus-stack` Helm chart (`third_party/kube-prometheus-stack/`) with all non-Grafana backend components disabled. This provides standalone Grafana alongside the maintained Kubernetes monitoring dashboards (`kubernetes-mixin`, `node-exporter-mixin`, etc.).

Similar to `prometheus/`, build-time chart templating inserts pseudo-variables (such as `${GCP_PROJECT_ID}`, `${CLOUD_ROBOTICS_DOMAIN}`, and `${CR_GF_*}`), which are substituted at deploy time in `cloud/grafana-operator.yaml`.

## Accessing Grafana Locally (Port-Forwarding)

Plain `kubectl port-forward` alone will fail to load frontend JS/CSS assets (`404 Not Found`) because:

* Grafana serves routes at root (`/`, `/public/...`, `/api/...`), relying on the upstream Ingress/Gateway rewrite rule (`cloud/grafana-http-route.yaml`) to strip the path prefix.
* However, `GF_SERVER_ROOT_URL` is configured with the external subpath (e.g. `https://<domain>/.../grafana`), causing Grafana to render `<base href=".../grafana/" />` in its HTML response.

To access the full Grafana web UI locally via `kubectl port-forward`, temporarily override `GF_SERVER_ROOT_URL` on the deployment before forwarding:

```bash
kubectl --context <context> -n app-grafana  set env deploy/grafana GF_SERVER_ROOT_URL=http://localhost:3000
kubectl --context <context> -n app-grafana port-forward svc/grafana 3000:80
xdg-open http://localhost:3000
```

Re-applying the chart or updating the `AppRollout` restores `GF_SERVER_ROOT_URL` automatically.

## Verifying Plugins and Datasources via CLI

You can also inspect installed plugins and test datasource connectivity (including GKE Workload Identity) directly inside the pod without browser access:

```bash
# List installed Grafana plugins
kubectl --context <context> -n app-grafana exec deploy/grafana -c grafana -- grafana cli plugins ls

# List configured datasources
kubectl --context <context> -n app-grafana exec deploy/grafana -c grafana -- curl -s -u <user:pw> http://localhost:3000/api/datasources

# Run backend health check for Google Cloud Trace
kubectl --context <context> -n app-grafana exec deploy/grafana -c grafana -- curl -s -u <user:pw> http://localhost:3000/api/datasources/uid/cloudtrace/health
```
