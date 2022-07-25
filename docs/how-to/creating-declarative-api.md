# Creating a declarative API

<!-- Estimated time: TODO -->

In this guide we will use a Kubernetes-style declarative API to interface to an external Charge Service for a robot.
This API is built around the concept of a ChargeAction resource, which instructs a robot to drive to a charger.
While the robot is charging, the status of the ChargeAction resource is kept up-to-date and can be observed.

## Motivation

RPC-based systems like ROS's [actionlib](http://wiki.ros.org/actionlib), while proven to be scalable, maintainable and useful, leave a few things to be desired:

1. **Synchronization**. The intent for a controller is stored in-memory in multiple components and we rely on correct synchronization.
For example, the motion planner sends the "turn wheel 3 times per second" message to the wheel actuator, then trusts that the wheel actuator will have received the intent and waits for it to act on the shared intent.
If a second process (such as an emergency stop) overwrites the intent of the wheel actuator, there's no standard channel to notify the motion planner.

2. **Persistence**. Since the intent is stored in-memory, it is lost when any process restarts.
This is the core reason that software in ROS systems can't be updated on the fly.

3. **Inspection**. For debugging, a coherent view into the current system intent would be great.
In RPC-based APIs, the intent is often updated differentially (eg "a little more to the left"), so our only hope of debugging is to log all messages ever sent.

In a declarative API, all actions and feedback are stored in a shared database&mdash;an approach built on Kubernetes' experience building robust distributed systems&mdash;which addresses these issues.
The latency added by going through the shared database means that
declarative APIs are best suited to latency-tolerant applications like
high-level control.

<!-- ## Concepts (describe resources, controllers, etc, and link to docs) -->

## Prerequisites

* Completed the [Quickstart Guide](../quickstart.md), after which the GCP project is set up and `gcloud-sdk` and `kubectl` are installed and configured.
* `docker` is installed and configured on the workstation ([instructions](https://docs.docker.com/install/linux/docker-ce/ubuntu/)).
<!-- * For the last part of the guide: A robot that has been successfully [connected to the cloud](connecting-robot.md). -->

Create a directory for the code examples of this guide, e.g.:

```shell
mkdir charge-service
cd charge-service
```

Set your GCP project ID as an environment variable:

```
export PROJECT_ID=[YOUR_GCP_PROJECT_ID]
```

All files created in this tutorial can be found in
[docs/how-to/examples/charge-service](https://github.com/googlecloudrobotics/core/tree/master/docs/how-to/examples/charge-service).
If you download the files, you have to replace the placeholder `[PROJECT_ID]` with your GCP project ID:

```shell
sed -i "s/\[PROJECT_ID\]/$PROJECT_ID/g" charge-controller.yaml
```

## Installing metacontroller

This tutorial is based on [metacontroller](https://metacontroller.github.io/metacontroller/intro.html), an add-on for Kubernetes that makes it easy to write and deploy [custom controllers](https://kubernetes.io/docs/concepts/extend-kubernetes/api-extension/custom-resources/#custom-controllers).
Custom controllers implement the logic behind a declarative API.

First, make sure that `kubectl` points to the correct GKE cluster:

```shell
kubectl config get-contexts
```

If the correct cluster is not marked with an asterisk in the output, you can switch to it with `kubectl config use-context [...]`.

Now install metacontroller to the cloud-cluster:

```shell
kubectl create namespace metacontroller
kubectl apply -f https://raw.githubusercontent.com/metacontroller/metacontroller/master/manifests/production/metacontroller-rbac.yaml
kubectl apply -f https://raw.githubusercontent.com/metacontroller/metacontroller/master/manifests/production/metacontroller-crds-v1.yaml
kubectl apply -f https://raw.githubusercontent.com/metacontroller/metacontroller/master/manifests/production/metacontroller.yaml
```

Let's check that all resources came up:

```console
> kubectl get pods --namespace metacontroller
NAME               READY   STATUS    RESTARTS   AGE
metacontroller-0   1/1     Running   0          1m
```

You can learn more details in the metacontroller's [install instructions](https://metacontroller.github.io/metacontroller/guide/install.html).

> **Limitations of metacontroller**:
> Writing custom controllers with metacontroller is easy, and you can use
> whatever programming language you prefer.
> However, it has some limitations.
>
> 1. metacontroller can't directly detect changes in external state, although
>    you can configure it to periodically reconcile your resources with external
>    systems. This introduces latency corresponding to the reconciliation
>    frequency.
> 1. The information available to your controller is limited.
>    You can't use metacontroller to create a controller that only acts on a
>    single resource out of many (for example, a controller that only executes
>    the highest-priority action).
>
> For advanced use cases, writing a controller in Golang offers more
> flexibility. See
> [sample-controller](https://github.com/kubernetes/sample-controller) for an
> example.

## Defining the controller logic

The core of a declarative API is the controller logic, which defines how the resources should be handled and reports the current status.
For the Charge Service, we've implemented the logic in a Python script.
Download [server.py](examples/charge-service/server.py):

```shell
curl -O https://raw.githubusercontent.com/googlecloudrobotics/core/master/docs/how-to/examples/charge-service/server.py
```

This Python program implements a server that listens on port 80 for incoming HTTP POST requests from metacontroller.
The controller logic is contained in the `sync()` method, which handles new ChargeActions by calling `charge_service.start_charging()`, and handles in-progress ChargeActions by updating the status.

[embedmd]:# (examples/charge-service/server.py python /.*state = current_status.get/ /return.*status.*/)
```python
    state = current_status.get("state", "CREATED")

    if state == "CREATED":
      # The ChargeAction has just been created. Use the external Charge Service
      # to start charging. Store the request ID in the status so we can use it
      # to check the state of the charge request.
      request_id = self.charge_service.start_charging()
      desired_status["state"] = "IN_PROGRESS"
      desired_status["request_id"] = request_id

    elif state == "IN_PROGRESS":
      try:
        # Get the progress of the charge request from the external service.
        progress = self.charge_service.get_progress(
            current_status["request_id"])
        desired_status["charge_level_percent"] = progress

        if progress == 100:
          # Charging has completed.
          desired_status["state"] = "OK"

      except ValueError as e:
        # The charge request was not found. This could be because the robot was
        # restarted during a charge, and the request was forgotten.
        desired_status["state"] = "ERROR"
        desired_status["message"] = str(e)

    elif state in ["OK", "CANCELLED", "ERROR"]:
      # Terminal state, do nothing.
      pass

    else:
      desired_status["state"] = "ERROR"
      desired_status["message"] = "Unrecognized state: %r" % state

    return {"status": desired_status, "children": []}
```

## Dockerizing the service

Next, to prepare our controller logic for deployment in the cloud, we package it as a Docker image. Make sure that the docker daemon is running and that your user has the necessary privileges:

```shell
docker run --rm hello-world
```

If this command fails, make sure Docker is installed according to the [installation instructions](https://docs.docker.com/install/linux/docker-ce/ubuntu/).

In the same directory as `server.py`, create a `Dockerfile` with the following contents:

[embedmd]:# (examples/charge-service/Dockerfile dockerfile)
```dockerfile
FROM python:alpine

WORKDIR /data

COPY server.py ./

CMD [ "python", "-u", "./server.py" ]
```

(Note: the `-u` option disables line-buffering; Python's line-buffering can prevent output from appearing immediately in the Docker logs.)

To build the Docker image, run:

```shell
docker build -t charge-controller .
```

You should see `Successfully tagged charge-controller:latest`. You can run the container locally with:

```shell
docker run -ti --rm -p 8000:8000 charge-controller
```

Then, from another terminal on the same workstation, send a request with an empty `parent` object:

```shell
curl -X POST -d '{"parent": {}, "children": []}' http://localhost:8000/
```

You should see a response like:

```json
{"status": {"state": "IN_PROGRESS", "request_id": "2423e70c-9dc7-47ac-abcb-b2ef0cbc676c"}, "children": []}
```

The response indicates that the controller would set `"state": "IN_PROGRESS"` on a newly-created ChargeAction.

## Uploading the Docker image to the cloud

In order to be able to run the server as a container in our cloud cluster, we need to upload the Docker image to our GCP project's private [container registry](https://cloud.google.com/container-registry/docs/pushing-and-pulling).

Enable the Docker credential helper:

```shell
gcloud auth configure-docker
```

Tag the image and push it to the registry:

```shell
docker tag charge-controller gcr.io/$PROJECT_ID/charge-controller
docker push gcr.io/$PROJECT_ID/charge-controller
```

The image should now show up in the [Container Registry](https://console.cloud.google.com/gcr).

## Deploying the declarative API in the cloud

Create a file called `charge-crd.yaml` with the following contents:

[embedmd]:# (examples/charge-service/charge-crd.yaml yaml)
```yaml
apiVersion: apiextensions.k8s.io/v1
kind: CustomResourceDefinition
metadata:
  name: chargeactions.example.com
  annotations:
    cr-syncer.cloudrobotics.com/spec-source: cloud
spec:
  group: example.com
  names:
    kind: ChargeAction
    plural: chargeactions
    singular: chargeaction
  scope: Namespaced
  versions:
    - name: v1alpha1
      served: true
      storage: true
      subresources:
        status: {}
      schema:
        openAPIV3Schema:
          type: object
          x-kubernetes-preserve-unknown-fields: true
---
apiVersion: rbac.authorization.k8s.io/v1
kind: ClusterRole
metadata:
  name: cloud-robotics:cr-syncer:chartaction
  labels:
    cr-syncer.cloudrobotics.com/aggregate-to-robot-service: "true"
rules:
- apiGroups:
  - example.com
  resources:
  - chargeactions
  verbs:
  - get
  - list
  - watch
  - update
```

This is a [custom resource](https://kubernetes.io/docs/concepts/extend-kubernetes/api-extension/custom-resources/) definition (CRD) for a resource called ChargeAction.
This simple example just describes the name and version of the API, but CRDs can also define schemas for the resources.
The ClusterRole configures [role-based access control](https://kubernetes.io/docs/reference/access-authn-authz/rbac/) to let the robot access the ChargeActions.
Don't worry about the `cr-syncer.cloudrobotics.com/spec-source` annotation for now, as it'll be explained later in the tutorial.

Next, create a file called `charge-controller.yaml` with the following contents, replacing `[PROJECT_ID]` with your GCP project ID:

[embedmd]:# (examples/charge-service/charge-controller.yaml yaml)
```yaml
apiVersion: metacontroller.k8s.io/v1alpha1
kind: CompositeController
metadata:
  name: charge-controller
spec:
  generateSelector: true
  parentResource:
    apiVersion: example.com/v1
    resource: chargeactions
  resyncPeriodSeconds: 1
  hooks:
    sync:
      webhook:
        url: http://charge-controller.default:8000/sync
---
apiVersion: v1
kind: Service
metadata:
  name: charge-controller
spec:
  selector:
    app: charge-controller
  ports:
  - port: 8000
---
apiVersion: apps/v1
kind: Deployment
metadata:
  name: charge-controller
spec:
  replicas: 1
  selector:
    matchLabels:
      app: charge-controller
  template:
    metadata:
      labels:
        app: charge-controller
    spec:
      containers:
      - name: controller
        image: gcr.io/[PROJECT_ID]/charge-controller
        ports:
        - containerPort: 8000
```

This file contains the information needed by Kubernetes and metacontroller to handle ChargeAction resources.
In the following, we will go over it bit by bit assuming basic familiarity with the [YAML format](https://en.wikipedia.org/wiki/YAML).

We define three Kubernetes resources:

* The *CompositeController* tells metacontroller to send ChargeAction resources to the charge-controller Service.
* The *Service* defines how the HTTP server is exposed within the cluster.
* The *Deployment* describes the Docker container to run.

Metadata, labels, and selectors are used to tie the three resources together.

A detailed explanation of the Kubernetes resources is out of scope for this guide, check out the [Kubernetes docs](https://kubernetes.io/docs/home/) or [metacontroller User Guide](https://metacontroller.github.io/metacontroller/guide.html) to get started.
There are a few points worth mentioning, though:

* In the Deployment, don't forget to replace `[PROJECT_ID]` with your GCP project ID.
* The CompositeController sets `resyncPeriodSeconds: 1`.
  This tells metacontroller to check each ChargeAction every second.
  This allows `server.py` to update the progress every second while the action is in progress.
* The CompositeController sets `url: http://charge-controller.default:8000/sync`.
  This tells metacontroller that the ChargeAction resources are handled by a service called `charge-controller` in the `default` namespace.

Deploy these resources by applying the configuration:

```shell
kubectl apply -f charge-crd.yaml
kubectl apply -f charge-controller.yaml
```

You can explore the various resources that were created on your cluster as a result of this command in the [GKE Console](https://console.cloud.google.com/kubernetes/workload) or with `kubectl`, e.g.:

```shell
kubectl get pods
```

The resulting list should contain a running pod with a name like `charge-controller-xxxxxxxxxx-xxxxx`.

## Redeploying after a change

If you make a change to `server.py`, you need to rebuild and push the Docker image:

```shell
docker build -t charge-controller .
docker tag charge-controller gcr.io/$PROJECT_ID/charge-controller
docker push gcr.io/$PROJECT_ID/charge-controller
```

The easiest way to get Kubernetes to restart the workload with the latest version of the container is to delete the pod:

```shell
kubectl delete pod -l 'app=charge-controller'
```

Kubernetes will automatically pull the newest image and recreate the pod.

If you make a change to `charge-controller.yaml`, all you have to do is apply it again:

```shell
kubectl apply -f charge-controller.yaml
```

## Accessing the API

You can use `kubectl` to interact with the API.
Create a file called `charge-action.yaml` with the following contents:

[embedmd]:# (examples/charge-service/charge-action.yaml yaml)
```yaml
apiVersion: example.com/v1
kind: ChargeAction
metadata:
  name: my-charge-action
```

Run the following command to create a ChargeAction and observe how its status changes:

```shell
kubectl apply -f charge-action.yaml \
  && watch -n0 kubectl describe chargeaction my-charge-action
```

Over the next 10 seconds, you should see the "Charge Level Percent" increase to 100, and then the state should become "CHARGED".

> **Troubleshooting**:
> If the ChargeAction's status doesn't change, check that metacontroller is installed by running `kubectl --namespace metacontroller get pods`.
> You should see `metacontroller-0   1/1    Running`.
> You can also check the metacontroller logs with `kubectl --namespace metacontroller logs metacontroller-0`


## Deploying the declarative API on the robot.

So far, the Charge Service has been running in the cloud, but we need to run
code on the robot to get it to charge.
We can change this with the `cr-syncer`, a component of Cloud Robotics Core that allows declarative APIs to work between Kubernetes clusters.
In particular, we can run the charge-controller on the robot, while creating the ChargeAction in the cloud cluster.
The `cr-syncer` takes care of copying the ChargeAction to the robot when the
robot has network connectivity.

**Prerequisite**: you'll need a robot that has been successfully [connected to the cloud](connecting-robot.md).

First, remove the controller from the cloud cluster:

```shell
# Note: run this on the workstation
kubectl delete -f charge-controller.yaml
```

Then SSH into the robot, install metacontroller, and bring up the charge-controller there:

```shell
# Note: run this on the robot
kubectl create namespace metacontroller
kubectl apply -f https://raw.githubusercontent.com/metacontroller/metacontroller/master/manifests/production/metacontroller-rbac.yaml
kubectl apply -f https://raw.githubusercontent.com/metacontroller/metacontroller/master/manifests/production/metacontroller-crds-v1.yaml
kubectl apply -f https://raw.githubusercontent.com/metacontroller/metacontroller/master/manifests/production/metacontroller.yaml

export PROJECT_ID=[YOUR_GCP_PROJECT_ID]
kubectl apply -f https://raw.githubusercontent.com/googlecloudrobotics/core/master/docs/how-to/examples/charge-service/charge-crd.yaml
curl https://raw.githubusercontent.com/googlecloudrobotics/core/master/docs/how-to/examples/charge-service/charge-controller.yaml \
  | sed "s/\[PROJECT_ID\]/$PROJECT_ID/g" | kubectl apply -f -
```

Now, check that these are running correctly:

```console
# Note: run this on the robot
> kubectl get pods --namespace metacontroller
NAME               READY   STATUS    RESTARTS   AGE
metacontroller-0   1/1     Running   0          1m
> kubectl get pods -l app=charge-controller
NAME                                 READY   STATUS    RESTARTS   AGE
charge-controller-57786849f8-xp5kf   1/1     Running   0          77s
```

Switch back to a terminal on your workstation.
As before, you can create a ChargeAction with `kubectl`, but this time it will be
handled by the controller on the robot.

```shell
# Note: run this on the workstation
kubectl delete -f charge-action.yaml
kubectl apply -f charge-action.yaml \
  && watch -n0 kubectl describe chargeaction my-charge-action
```

How does this work?

- The `cr-syncer` runs on the robot and watches custom resources in the cloud.
- It sees the `cr-syncer.cloudrobotics.com/spec-source: cloud` annotation on the
  CustomResourceDefinition, which tells it to copy the `spec` from
  `my-charge-action` in the cloud cluster into a copy of `my-charge-action` in
  the robot cluster.
- While the robot is charging, the robot's charge-controller updates the status
  in the robot's cluster.
- The `cr-syncer` copies the status back up to the original resource in the
  cloud cluster.

## Cleaning up

In order to stop the controller and remove the CRD you created, run:

```shell
kubectl delete -f charge-controller.yaml -f charge-crd.yaml
```

If you want to uninstall metacontroller too, run:

```shell
kubectl delete namespace metacontroller
```

If you installed on the robot, you'll need to run these commands there too.

<!--
TODO(rodrigoq): define "What's Next" for declarative APIs

## What's next

There are a few inconvenient steps in this guide, e.g.:

* manually replacing the project ID in all source files or
* remotely logging in to the robot to start a workload.

This is where the app management layer comes in; it provides, among other capabilities:

* ways of bundling and parameterizing Kubernetes resources and
* remote management of Kubernetes resources/workloads on the robot.

Also note that both the server and the client side can be implemented with similar ease in other programming languages, such as [Java](https://developers.google.com/api-client-library/java/) or [Go](https://github.com/googleapis/google-api-go-client).
-->
