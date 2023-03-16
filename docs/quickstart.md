# Quickstart

Estimated time: 10 min

This page describes how to set up a Google Cloud Platform (GCP) project
containing the Cloud Robotics Core (CRC) components.
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

    ```shell
    gcloud components install kubectl gke-gcloud-auth-plugin
    ```

    If you're using Cloud Shell, Debian, or Ubuntu, you may need to use apt instead:

    ```shell
    apt-get install kubectl google-cloud-sdk-gke-gcloud-auth-plugin
    ```

1. Install tools required for installation:

    ```shell
    sudo apt-get install curl tar xz-utils
    ```

## Deploy the project

1. Create application default credentials, which are used to deploy the cloud project.

    ```shell
    gcloud auth application-default login
    ```

1. Create a directory for CRC installer.

    ```shell
    mkdir cloud-robotics
    cd cloud-robotics
    ```

1. Set your GCP project ID as an environment variable.

    ```shell
    export PROJECT_ID=[YOUR_GCP_PROJECT_ID]
    ```

1. Install the latest nightly build into your GCP project by running the install script.
    Accept the default configuration by hitting `ENTER` on all questions; you can change the settings later.

    ```shell
    curl -fS "https://storage.googleapis.com/cloud-robotics-releases/run-install.sh" >run-install.sh
    bash ./run-install.sh $PROJECT_ID
    ```

The install script created a Kubernetes cluster using Google Kubernetes Engine
and used [Synk][synk] to install the Cloud Robotics Core component helm charts.
You can browse these components on the [Workloads dashboard][workloads].
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

In addition to the cluster, the install script also created:

* the [[PROJECT_ID]-cloud-robotics-config bucket][storage-bucket], containing a `config.sh` and a Terraform state which are necessary to update your cloud project later,
* the [[PROJECT_ID]-robot Cloud Storage bucket][storage-bucket], containing the scripts that connect robots to the cloud, and
* the [Identity & Access Management policies][iam] that authorize robots and humans to communicate with GCP.

## Update the project

To update your Cloud Robotics configuration, run the install script with the `--set-config` flag.

```shell
bash ./run-install.sh $PROJECT_ID --set-config
```

This command only updates the config but does not update your cloud project.
To update the installation to the latest version and apply config changes, run the installer again.

```shell
bash ./run-install.sh $PROJECT_ID
```

If you deleted the install scipt or you want to run an update from another machine which has the Cloud SDK installed, simply run:

```
curl -fS "https://storage.googleapis.com/cloud-robotics-releases/run-install.sh"\
    | bash -s -- $PROJECT_ID
```

## Clean up

The following command will delete:

* the [cloud-robotics Kubernetes cluster](https://console.cloud.google.com/kubernetes/list)

This can be useful if the cluster is in a broken state.
Be careful with this invocation, since you'll have to redeploy the project and reconnect any robots afterwards.

```shell
curl -fS "https://storage.googleapis.com/cloud-robotics-releases/run-install.sh"\
    | bash -s -- $PROJECT_ID --delete
```

> **Known issue** After deleting CRC from your project, the endpoint services will be in a "pending deletion" state for 30 days.
> If you want to reinstall CRC into the same project again, you have to [undelete the services][undelete-service] manually.

If you want to completely shut down the project, see [the Resource Manager documentation][shutting_down_projects].

## Next steps

* [Connect a robot to the cloud](how-to/connecting-robot.md).
* [Set up OAuth](how-to/setting-up-oauth.md)


[resource-manager]: https://console.cloud.google.com/cloud-resource-manager
[modify-project]: https://cloud.google.com/billing/docs/how-to/modify-project
[cloud-sdk]: https://cloud.google.com/sdk/docs/
[workloads]: https://console.cloud.google.com/kubernetes/workload
[storage-bucket]: https://console.cloud.google.com/storage/browser
[iam]: https://console.cloud.google.com/iam-admin/iam
[undelete-service]: https://cloud.google.com/sdk/gcloud/reference/endpoints/services/undelete
[shutting_down_projects]: https://cloud.google.com/resource-manager/docs/creating-managing-projects#shutting_down_projects
[synk]: https://github.com/googlecloudrobotics/core/tree/master/src/go/cmd/synk/README.md
