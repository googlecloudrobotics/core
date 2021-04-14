# Running a ROS node as a Kubernetes deployment

Estimated time: 10 min

The following instructions describe how to setup a Kubernetes cluster on a robot
running Ubuntu 20.04 and run a ROS node on it.

The installation script installs and configures:

* Docker
* A single-node Kubernetes cluster (packages: kubectl, kubeadm, kubelet)

Once you've done this, you can use Kubernetes to:

* Reduce downtime during updates with Kubernetes deployments
* Apply CPU, disk or memory quotas to individual processes
* Add additional compute nodes to the cluster, such as an Nvidia Jetson
* Use a network plugin to apply network access control
* Manage project configuration or sensitive secrets such as account credentials

For more details, refer to the [Kubernetes documentation](https://kubernetes.io/docs/home/).

## Installing the cluster on the robot

Download and run [install\_k8s\_on\_robot.sh](https://raw.githubusercontent.com/googlecloudrobotics/core/master/src/bootstrap/robot/install_k8s_on_robot.sh). This script will take a few minutes as it downloads and installs the dependencies of the Kubernetes cluster.

```shell
$ curl https://raw.githubusercontent.com/googlecloudrobotics/core/master/src/bootstrap/robot/install_k8s_on_robot.sh | bash
[...]
The local Kubernetes cluster has been installed.
```

After the script successfully finishes, the Kubernetes cluster is up and running.

> **Note:** You might notice instructions for creating `~/.kube/config`, deploying a pod network, and joining nodes to the cluster. You can ignore these instructions for now, as the script has already set up a single-node cluster.

## Run a ROS node with Kubernetes

If you're already using ROS on your robot, you can run a ROS node inside Kubernetes that will communicate with other nodes on the robot. If not, you can follow the [ROS tutorials](http://wiki.ros.org/ROS/Tutorials) to get started.

First, make sure you're running `roscore`. In another terminal, please run:

```shell
roscore
```

> **Caution:** If you have a more complicated ROS setup, such as a ROS master running on another machine, you might need to change `ROS_MASTER_URI` or `ROS_IP` in rostopic-echo.yaml.

You can run a ROS node by creating a Kubernetes Deployment object, and you can describe a Deployment in a YAML file.
For example, this YAML file describes a Deployment that runs `rostopic echo`.
Create file called `rostopic-echo.yaml` with the following contents:

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: rostopic-echo
spec:
  selector:
    matchLabels:
      app: rostopic-echo
  template:
    metadata:
      labels:
        app: rostopic-echo
    spec:
      containers:
      - name: rostopic-echo
        image: ros:melodic-ros-core
        args:
        - rostopic
        - echo
        - chatter
        env:
        - name: ROS_MASTER_URI
          value: http://192.168.9.1:11311
        - name: ROS_IP
          value: 192.168.9.1
      hostNetwork: true
```

> **Note:** For simplicity, this example uses `hostNetwork: true` to disable network isolation. Advanced users can disable host networking to improve security. For more information, see the networking documentation for <a href="http://wiki.ros.org/ROS/NetworkSetup">Docker</a>, <a href="https://kubernetes.io/docs/concepts/cluster-administration/networking/">Kubernetes</a> and <a href="http://wiki.ros.org/ROS/NetworkSetup">ROS</a>.

After creating `rostopic-echo.yaml`, use `kubectl` to apply it to your cluster:

```shell
kubectl apply -f rostopic-echo.yaml
```

Depending on your internet connection, it will take a minute or so to download the Docker image. Wait until you see `Running`:

```console
$ watch kubectl get pods -l app=rostopic-echo
NAMESPACE     NAME                                        READY   STATUS      RESTARTS   AGE
default       rostopic-echo-576cbf47c7-dtlc6              1/1     Running     0          1m
```

Now, publish a ROS message and check that it was received inside Kubernetes:

```console
$ rostopic pub -1 chatter std_msgs/String "Hello, world"
$ kubectl logs -l app=rostopic-echo
data: "Hello, world"
---
```

Kubernetes will keep this node running until you delete the deployment:

```shell
kubectl delete -f rostopic-echo.yaml
```
