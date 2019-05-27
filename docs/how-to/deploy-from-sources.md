# Deploy Cloud Robotics Core from sources

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

    ```shell
    gcloud components install kubectl
    ```

    If you're using Debian or Ubuntu, you may need to use `apt install kubectl` instead.

1. [Install the Bazel build system][install-bazel].

1. Install additional build dependencies:

    ```shell
    sudo apt-get install default-jdk git python-dev unzip xz-utils
    ```

[resource-manager]: https://console.cloud.google.com/cloud-resource-manager
[modify-project]: https://cloud.google.com/billing/docs/how-to/modify-project
[cloud-sdk]: https://cloud.google.com/sdk/docs/
[install-bazel]: https://github.com/bazelbuild/bazel/blob/0.22.0/site/docs/install-ubuntu.md

## Build and deploy the project

1. Clone the source repo.

    ```shell
    git clone https://github.com/googlecloudrobotics/core
    cd core
    ```

1. Create application default credentials, which are used to deploy the cloud project and
   authorize access to the cloud docker registry.

    ```shell
    gcloud auth application-default login
    gcloud auth configure-docker
    ```

1. Create a Cloud Robotics config in your project:

    ```shell
    ./deploy.sh set_config [PROJECT_ID]
    ```

    You can keep the defaults for the other settings by hitting `ENTER`.

    This command creates a file `config.sh` containing your choices and stores
    in into a cloud-storage bucket named `[PROJECT_ID]-cloud-robotics-config`.
    You can verify the settings using:

    ```shell
    gsutil cat gs://[PROJECT_ID]-cloud-robotics-config/config.sh
    ```


1. Build the project. Depending on your computer and internet connection, it may take around 15 minutes.

    ```shell
    bazel build //...
    ```

1. Deploy the cloud project.

    ```shell
    ./deploy.sh create [PROJECT_ID]
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

```console
$ kubectl get pods

NAME                READY   STATUS             RESTARTS   AGE
cert-manager-xxx    1/1     Running            0          1m
nginx-ingress-xxx   1/1     Running            0          1m
oauth2-proxy-xxx    0/1     CrashLoopBackOff   4          1m
token-vendor-xxx    1/1     Running            0          1m
```

> **Note** Unless you already set up OAuth, the `oauth2-proxy` will show an error which we will ignore for now.


In addition to the cluster, `deploy.sh` also created:

* the [cloud-robotics IoT Core Registry](https://console.cloud.google.com/iot/registries) that will be used to manage the list of authorized robots,
* the [[PROJECT_ID]-robot Cloud Storage bucket](https://console.cloud.google.com/storage/browser), containing the scripts that connect robots to the cloud, and
* the [Identity & Access Management policies](https://console.cloud.google.com/iam-admin/iam) that authorize robots and humans to communicate with GCP.

With the project deployed, you're ready to [connect a robot to the cloud](how-to/connecting-robot.md).

## Update the project

To apply changes made in the source code, run:

```shell
./deploy.sh update [PROJECT_ID]
```

## Clean up

The following command will delete:

* the [cloud-robotics Kubernetes cluster](https://console.cloud.google.com/kubernetes/list)
* the [cloud-robotics IoT Core Registry](https://console.cloud.google.com/iot/registries)

This can be useful if the cluster is in a broken state.
Be careful with this invocation, since you'll have to redeploy the project and reconnect any robots afterwards.

```shell
./deploy.sh delete [PROJECT_ID]
```

If you want to completely shut down the project, see [the Resource Manager documentation](https://cloud.google.com/resource-manager/docs/creating-managing-projects#shutting_down_projects).

## What's next

* [Connecting a robot to the cloud](how-to/connecting-robot.md).
* [Setting up OAuth for web UIs](how-to/setting-up-oauth.md).
