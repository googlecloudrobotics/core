# Project configuration

The project configuration that one has entered during the initial setup is
stored with the project in GCS. One can look at the options with the following
command:

```shell
gsutil cat gs://${PROJECT_ID}-cloud-robotics-config/config.sh
```

The settings contained in the config file are used by terraform to setup the
project infrastructure and used by the cloud and chart-assignment-controller services running
in kubernetes to configure apps.

The terraform support is encapsulated in deploy.sh that creates a temporary
`terraform.tfvars` file.

To support configuring apps, we pass the settings to app-rollout-controller where they are
provided as additional variables for helm templating. The command below prints
the settings we pass to app-rollout-controller:

```shell
kubectl get deployment app-rollout-controller -o=jsonpath='{.spec.template.spec.containers[0].args[0]}'
```

