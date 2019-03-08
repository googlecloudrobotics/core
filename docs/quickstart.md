# Quickstart

Estimated time: 10 min

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
1. [Install the Cloud SDK][cloud-sdk]. When prompted, choose the project you created above.
1. After installing the Cloud SDK, install the `kubectl` command-line tool:

    ```
    gcloud components install kubectl
    ```

    If you're using Debian or Ubuntu, you may need to use `apt-get install kubectl` instead.

1. Install tools required for installation:

    ```
    sudo apt-get install curl jq tar xz-utils
    ```

# Deploy the project

1. Create application default credentials, which are used to deploy the cloud project.

    ```
    gcloud auth application-default login
    ```

1. Create a directory for the project configuration.

    ```
    mkdir cloud-robotics
    cd cloud-robotics
    ```

1. Install the latest nightly build into the GCP project you are using. The command below will interactively ask you for the ID of your GCP project.

    ```
    curl -fS "https://storage.googleapis.com/cloud-robotics-releases/run-install.sh" >run-install.sh
    bash ./run-install.sh
    ```

The install script created a Kubernetes cluster using Google Kubernetes Engine and used Helm to install the Cloud Robotics Core components.
You can browse these components on the [Workloads dashboard][workloads].
Alternatively, you can list them from the console on your workstation:

```
$ kubectl get pods

NAME                READY   STATUS             RESTARTS   AGE
cert-manager-xxx    1/1     Running            0          1m
nginx-ingress-xxx   1/1     Running            0          1m
oauth2-proxy-xxx    0/1     CrashLoopBackOff   4          1m
token-vendor-xxx    1/1     Running            0          1m
```

> **Note** Unless you already set up OAuth, the `oauth2-proxy` will show an error which we will ignore for now.

In addition to the cluster, the install script also created:

* the [cloud-robotics IoT Core Registry][iot-registry] that will be used to manage the list of authorized robots,
* the [[PROJECT_ID]-robot Cloud Storage bucket][storage-bucket], containing the scripts that connect robots to the cloud, and
* the [Identity & Access Management policies][iam] that authorize robots and humans to communicate with GCP.


# Update the project

To update the installation to a newer version run the installer again.
```
curl -fS "https://storage.googleapis.com/cloud-robotics-releases/run-install.sh" | bash
```

## Clean up

The following command will delete:

* the [cloud-robotics Kubernetes cluster](https://console.cloud.google.com/kubernetes/list)
* the [cloud-robotics IoT Core Registry](https://console.cloud.google.com/iot/registries)

This can be useful if the cluster is in a broken state.
Be careful with this invocation, since you'll have to redeploy the project and reconnect any robots afterwards.

```
cd cloud-robotics-core
./deploy.sh delete
```

If you want to completely shut down the project, see [the Resource Manager documentation](https://cloud.google.com/resource-manager/docs/creating-managing-projects#shutting_down_projects).

# Next steps

* [Connect a robot to the cloud](how-to/connecting-robot.md).
* [Set up OAuth](how-to/setting-up-oauth.md)


[resource-manager]: https://console.cloud.google.com/cloud-resource-manager
[modify-project]: https://cloud.google.com/billing/docs/how-to/modify-project
[cloud-sdk]: https://cloud.google.com/sdk/docs/
[workloads]: https://console.cloud.google.com/kubernetes/workload
[iot-registry]: https://console.cloud.google.com/iot/registries
[storage-bucket]: https://console.cloud.google.com/storage/browser
[iam]: https://console.cloud.google.com/iam-admin/iam
