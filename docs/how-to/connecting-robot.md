# Connecting a robot to the cloud

Estimated time: 10 min

This page describes how to connect a Kubernetes cluster on a robot to the cloud (tested on Ubuntu 14, 16 and 18).

Once you've done this, you can:

* Run a private Docker container from the Google Container Registry
* Securely communicate with cloud services
* See logs from the robot in the Cloud Console

## Setting up the GCP project

1. If you haven't already, complete the [Setting up the GCP project](../quickstart.md) steps.

1. On the computer you used to set up the cloud project, generate an access token, which you'll use to give the robot access to the cloud:

    ```shell
    gcloud auth application-default print-access-token
    ```

## Installing the cluster on the robot

You'll need to install a Kubernetes cluster on the robot before you can connect it to the cloud. The cluster manages and supports the processes that communicate with the cloud.

The installation script installs and configures:

* Docker
* A single-node Kubernetes cluster (packages: kubectl, kubeadm, kubelet)
* Deployments in the local cluster that handle authentication and upload logs to [Stackdriver Logging](https://cloud.google.com/logging/)

<!-- this comment is required to separate the lists -->

1. Download and run [install\_k8s\_on\_robot.sh](https://raw.githubusercontent.com/googlecloudrobotics/core/master/src/bootstrap/robot/install_k8s_on_robot.sh). This script will take a few minutes as it downloads and installs the dependencies of the Kubernetes cluster.

    ```console
    $ curl https://raw.githubusercontent.com/googlecloudrobotics/core/master/src/bootstrap/robot/install_k8s_on_robot.sh | bash
    [...]
    The local Kubernetes cluster has been installed.
    ```

    After the script successfully finishes, the Kubernetes cluster is up and running.

    > **Note:**  At the end of the script output, you might notice instructions for creating `~/.kube/config`, deploying a pod network, and joining nodes to the cluster. You can ignore these instructions for now, as the script has already set up a single-node cluster.

1. Set up the robot cluster to connect to the cloud. When running `setup_robot.sh`, you'll need to enter the access token you generated earlier. You may find it easiest if you SSH into the robot from the workstation you used to set up the project.

    ```shell
    mkdir -p ~/cloud-robotics-core
    cd ~/cloud-robotics-core
    curl https://storage.googleapis.com/[PROJECT_ID]-robot/setup_robot.sh >setup_robot.sh
    bash setup_robot.sh my-robot --project [PROJECT_ID] \
      --robot-type my-robot-type
    ```

    Replace `[PROJECT_ID]` with your GCP project ID. When prompted for an access token, provide the authentication token you generated earlier.

    > **Note:** `my-robot-type` is a placeholder and you can ignore it for now.

## What's next

* [Using Cloud Storage from a robot](using-cloud-storage.md).

## Uninstalling the local cluster

You can remove the local cluster with the following command:

```shell
sudo kubeadm reset
```

You may also want to remove the following APT packages and repositories, which `install_k8s_on_robot.sh` installs if they are not present:

```shell
sudo apt-get purge kubectl kubelet kubeadm
sudo rm /etc/apt/sources.list.d/kubernetes.list
sudo apt-get purge docker-ce
sudo add-apt-repository --remove \
  "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
```
