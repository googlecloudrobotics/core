# Deploying a gRPC service written in C++

Estimated time: 60 min

In this guide we will deploy a gRPC service written in C++ and deploy it to our Google Kubernetes Engine (GKE) cluster in the cloud in such a way that authentication is required for access. We will show how to access the service from the workstation and how to access it from code running in the robot's Kubernetes cluster.


## Prerequisites

* Completed the [Quickstart Guide](../quickstart.md), after which the GCP project is set up and `gcloud-sdk` and `kubectl` are installed and configured.
* `docker` is installed and configured on the workstation ([instructions](https://docs.docker.com/install/linux/docker-ce/ubuntu/)).
* `git` is installed on the workstation.
* For the last part of the guide: A robot that has been successfully [connected to the cloud](connecting-robot.md).

All files for this tutorial are located in
[docs/how-to/examples/greeter-service/](https://github.com/googlecloudrobotics/core/tree/master/docs/how-to/examples/greeter-service).

```shell
git clone https://github.com/googlecloudrobotics/core
cd core/docs/how-to/examples/greeter-service
```

Set your GCP project ID as an environment variable:

```shell
export PROJECT_ID=[YOUR_GCP_PROJECT_ID]
```


## Running gRPC server and client locally

We will use [gRPC's quickstart example](https://grpc.io/docs/quickstart/cpp.html) with small modifications.
If you like to learn more about gRPC in C++, follow their guide first.

The gRPC `helloworld.Greeter` service is defined in `proto/helloworld.proto`.
It accepts a `HelloRequest` containing a `name` and responds with a `HelloReply` containing a `message`.
The server is implemented in `server/server.cc` and the client is implemented `client/client.cc`. The client sends the request with `name: "world"` to the server which responds with `message: "Hello <name>"`.

In this tutorial, we build the server and client code inside Docker containers, so you don't need to install the gRPC library.
If you prefer, you can install the gRPC following [these instructions](https://github.com/grpc/grpc/blob/master/src/cpp/README.md) and build the server and client locally using the provided `Makefile`.

Make sure the Docker daemon is running and your user has the necessary privileges:

```shell
docker run --rm hello-world
```

If this command fails, make sure Docker is installed according to the [installation instructions](https://docs.docker.com/install/linux/docker-ce/ubuntu/).

The Docker image for the server is configured in `server/Dockerfile`:

[embedmd]:# (examples/greeter-service/server/Dockerfile dockerfile)
```dockerfile
FROM grpc/cxx:1.12.0

WORKDIR /data

COPY server/server.cc ./server/
COPY proto/helloworld.proto ./proto/
COPY Makefile ./

RUN make greeter-server && make clean

CMD ["./greeter-server"]
```

We use the [grpc/cxx](https://hub.docker.com/r/grpc/cxx) Docker image which contains all the build tools and libraries (`g++`, `make`, `protoc`, and `grpc`) we need to build the `greeter-server` binary.
The Docker image for the client is configured in `client/Dockerfile` which builds the `greeter-client` from `client/client.cc`.

To build the Docker images, run:

```shell
docker build -t greeter-server -f server/Dockerfile .
docker build -t greeter-client -f client/Dockerfile .
```

> **Note**
> The docker files are in the subfolders `greeter-server/server/` and `greeter-server/client`, but the docker command must be called from `greeter-server/` to include the files which are shared between the server and the client.

You should now have an image tagged `greeter-server` and one tagged `greeter-client` in your local registry:

```shell
docker images | grep greeter
```

To run the server locally, the container's port 50051, which specified as gRPC port in `server.cc`, has to be published to your machine with the flag `-p 50051:50051`:

```shell
docker run --rm -p 50051:50051 --name greeter-server greeter-server
```

In another console run the client container.
The flag `--network=host` tells the container to use your workstation's network stack which allows the client to connect to `localhost`.

```shell
docker run --rm --network=host greeter-client ./greeter-client localhost
```

You should see `Greeter received: Hello world` in the client's output and `Received request: name: "world"` in the server's output. You can also send your own name in the gRPC request to the server, try:

```shell
docker run --rm --network=host greeter-client \
  ./greeter-client localhost $USER
```

You can stop the server from another terminal by running:

```shell
docker stop greeter-server
```


## Uploading the Docker image to the cloud

In order to be able to run the server as a container in our cloud cluster, we need to upload the Docker image to our GCP project's private [container registry](https://cloud.google.com/container-registry/docs/pushing-and-pulling).

Enable the Docker credential helper:

```shell
gcloud auth configure-docker
```

Tag the image and push it to the registry:

```shell
docker tag greeter-server gcr.io/$PROJECT_ID/greeter-server
docker push gcr.io/$PROJECT_ID/greeter-server
```

The image should now show up in the [container registry](https://console.cloud.google.com/gcr).


## Deploying the service in the cloud using Kubernetes

Run the following command to create `greeter-server.yaml` using the provided template:

```shell
cat greeter-server.yaml.tmpl | envsubst >greeter-server.yaml
```

This file contains the information needed by Kubernetes to run the gRPC service in our cloud cluster.
The three resources, Ingress, Service, and Deployment, are explained in the [deploying a service tutorial](deploying-service.md).
In contrast to the other tutorial, the Ingress tells nginx to forward incoming requests to a gRPC backend.

[embedmd]:# (examples/greeter-service/greeter-server.yaml.tmpl yaml /^/ /---/)
```yaml
apiVersion: networking.k8s.io/v1beta1
kind: Ingress
metadata:
  name: greeter-server-ingress
  annotations:
    kubernetes.io/ingress.class: "nginx"
    nginx.ingress.kubernetes.io/backend-protocol: GRPC
    nginx.ingress.kubernetes.io/auth-url: "http://token-vendor.default.svc.cluster.local/apis/core.token-vendor/v1/token.verify?robots=true"
spec:
  rules:
  - host: "www.endpoints.${PROJECT_ID}.cloud.goog"
    http:
      paths:
      - # must match the namespace and service name in the proto
        path: /helloworld.Greeter/
        backend:
          serviceName: greeter-server-service
          # must match the port used in server.cc
          servicePort: 50051
---
```

Make sure that `kubectl` points to the correct GCP project:

```shell
kubectl config get-contexts
```

If the correct cluster is not marked with an asterisk in the output, you can switch contexts with `kubectl config use-context [...]`.)
Then deploy by applying the configuration:

```shell
kubectl apply -f greeter-server.yaml
```

You can explore the various resources that were created on your cluster as a result of this command in the [GKE Console](https://console.cloud.google.com/kubernetes/workload) or with `kubectl`, e.g.:

```shell
kubectl get pods
```

The resulting list should contain a running pod with a name like `greeter-server-xxxxxxxxxx-xxxxx`.


## Redeploying after a change

For convenience, `deploy.sh` provides some commands to create, delete, and update the service.
If you make changes to `greeter-server.yaml.tmpl`, all you have to do is run:

```shell
./deploy.sh update_config
```

If you make changes to `server.cc`, you need to run:

```shell
./deploy.sh update_server
```

This builds, tags, and pushes the Docker image, and then forces a redeployment of the image by calling `kubectl delete pod -l 'app=greeter-server-app'`.
It also updates the resource definitions, so you don't have to run `./deploy.sh update_config` if you made changes to both files.


## Accessing the API

In `client/client.cc` we use `grpc::InsecureChannelCredentials()` when talking to `localhost` while we use `grpc::GoogleDefaultCredentials()` when talking to any other address.
SSL authentication with credentials from the user or robot are necessary when talking to the `greeter-server` in the Cloud Robotics project.

[embedmd]:# (examples/greeter-service/client/client.cc c++ /^ +if.*localhost/ /^ +}$/)
```c++
  if (grpc_endpoint.find("localhost:") == 0 ||
      grpc_endpoint.find("127.0.0.1:") == 0) {
    channel_creds = grpc::InsecureChannelCredentials();
  } else {
    channel_creds = grpc::GoogleDefaultCredentials();
  }
```

Let's try to access our server.
We have to connect to the nginx ingress which is hosted on `www.endpoints.$PROJECT_ID.cloud.goog:443`.
To ensure we have valid credentials to talk to nginx we have to mount our `~/.config` folder in the container.

```shell
docker run --rm -v ~/.config:/root/.config greeter-client \
  ./greeter-client www.endpoints.$PROJECT_ID.cloud.goog:443 workstation
```

Recall that when running `./greeter-server` on your workstation you were able to see the server's log output upon receiving a request.
This log output is also recorded when the server is running in the cloud cluster. To inspect it, run:

```shell
kubectl logs -l 'app=greeter-server-app'
```

Or go to the [GKE Console](https://console.cloud.google.com/kubernetes/workload), select the `greeter-server` workload and click on "Container logs".


## Accessing the API from the robot

In order to run `greeter-client` on the robot's Kubernetes cluster, we again package it as a Docker image and push it to our container registry, to which the robot also has access.
Our deploy script offers a command to build, tag, and push the image to the cloud registry, like we did with the server container:

```shell
./deploy.sh push_client
```

And finally, to execute the script, SSH into robot and run:

```shell
export PROJECT_ID=[YOUR_GCP_PROJECT_ID]
docker pull grpc/cxx:1.12.0  # This may take several minutes, depending on WiFi connection
kubectl run -ti --rm --restart=Never --image=gcr.io/$PROJECT_ID/greeter-client greeter-client \
  -- ./greeter-client www.endpoints.$PROJECT_ID.cloud.goog:443 robot
```

You should see the server's answer `Hello robot`.

Two things are noteworthy:

* The `greeter-client` Docker image was pulled from the container registry without the need for additional credentials. This worked because there is a periodical job running on the robot's Kubernetes cluster that refreshes the GCR credentials. Run `kubectl get pods` on the robot and you will see pod names that start with `gcr-credential-refresher`.
* `grpc::GoogleDefaultCredentials()` in the client's code automatically obtained credentials that allowed the robot to access the cloud cluster. This worked because the the local Metadata Server obtains access tokens for the robot in the background.


## Cleaning up

In order to stop the service in the cloud cluster and revert the configuration changes, run:

```shell
./deploy.sh delete
```
