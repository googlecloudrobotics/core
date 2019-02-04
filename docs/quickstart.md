# Quickstart

Estimated time: 30 min

This page describes how to set up a Google Cloud Platform (GCP) project
containing the Cloud Robotics Core components.
In particular, this creates a cluster with Google Kubernetes Engine and prepares
it to accept connections from robots, which enables those robots to securely
communicate with GCP.
The commands were tested on machines running Debian (Stretch) or Ubuntu (16.04
and 18.04) Linux.

1. In the GCP Console, go to the [Manage resources][resource-manager] page and
   select or create a project.
1. Make sure that [billing][modify-project] is enabled for your project.
1. [Install the Cloud SDK][cloud-sdk]. When prompted, choose the project that you created above.
1. After installing the Cloud SDK, install the `kubectl` command-line tool:

    ```
    gcloud components install kubectl
    ```

    If you're using Debian or Ubuntu, you may need to use `apt install kubectl` instead.

1. [Install the Bazel build system][install-bazel].

1. Install additional build dependencies:

    ```
    sudo apt-get install default-jdk git jq python-dev unzip
    ```

[resource-manager]: https://console.cloud.google.com/cloud-resource-manager
[modify-project]: https://cloud.google.com/billing/docs/how-to/modify-project
[cloud-sdk]: https://cloud.google.com/sdk/docs/
[install-bazel]: https://github.com/bazelbuild/bazel/blob/0.22.0/site/docs/install-ubuntu.md

## Build and deploy the project

1. Clone the source repo.

    ```
    git clone https://github.com/googlecloudrobotics/core
    cd core
    ```

1. Import the GCP project you are using, replacing `[PROJECT_ID]` with its project ID.

    ```
    ./deploy.sh set-project [PROJECT_ID]
    ```

    This command creates two files:

    * `config.bzl`, which configures the build system to store its outputs in the Container Registry, and
    * `config.sh`, which configures the deployment.

1. Optionally, edit `config.sh` to change the Compute Engine [region and zone](https://cloud.google.com/compute/docs/regions-zones/) from the default value of `europe-west1`.

1. Build the project. Depending on your computer and internet connection, it may take around 15 minutes.

    ```
    bazel build //...
    ```

1. Create application default credentials, which are used to deploy the cloud project.

    ```
    gcloud auth application-default login
    ```

1. Deploy the cloud project.

    ```
    ./deploy.sh create
    ```

> **Known issue:**
> Sometimes, this command fails with an error message like
> `Error 403: The caller does not have permission`,
> `Bad status during token exchange: 503`, or
> `Error enabling service`.
> In these cases, wait for a minute and try again.

`deploy.sh` created a Kubernetes cluster using Google Kubernetes Engine and used Helm to install the Cloud Robotics Core components.
You can browse these components on the [Workloads dashboard](https://console.cloud.google.com/kubernetes/workload).
Alternatively, you can list them from the console on your workstation:

```
$ kubectl get pods
NAME                                        READY   STATUS    RESTARTS   AGE
cert-manager-7d4bfc44ff-fdrkl               1/1     Running   0          1m
nginx-ingress-controller-64ffff8d4b-4hzb8   1/1     Running   0          1m
oauth2-proxy-67569d4c94-4jgnw               1/1     Running   0          1m
token-vendor-69b4494866-lvcgf               1/1     Running   0          1m
```

In addition to the cluster, `deploy.sh` also created:

* the [cloud-robotics IoT Core Registry](https://console.cloud.google.com/iot/registries) that will be used to manage the list of authorized robots,
* the [-robot Cloud Storage bucket](https://console.cloud.google.com/storage/browser), containing the scripts that connect robots to the cloud, and
* the [Identity & Access Management policies](https://console.cloud.google.com/iam-admin/iam) that authorize robots and humans to communicate with GCP.

With the project deployed, you're ready to [connect a robot to the
cloud](how-to/connecting-robot.md).

## Update the project

To apply changes made in the source code, run:

```
./deploy.sh update
```

## Clean up

The following command will delete:

* the [cloud-robotics Kubernetes cluster](https://console.cloud.google.com/kubernetes/list)
* the [cloud-robotics IoT Core Registry](https://console.cloud.google.com/iot/registries)

This can be useful if the cluster is in a broken state.
Be careful with this invocation, since you'll have to redeploy the project and reconnect any robots afterwards.

```
./deploy.sh delete
```

If you want to completely shut down the project, see [the Resource Manager documentation](https://cloud.google.com/resource-manager/docs/creating-managing-projects#shutting_down_projects).

## What's next

* [Connecting a robot to the cloud](how-to/connecting-robot.md).
* [Setting up OAuth for web UIs](how-to/setting-up-oauth.md).
