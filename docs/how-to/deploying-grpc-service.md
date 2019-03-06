# Deploying a gRPC service written in C++

Estimated time: 60 min

In this guide we will deploy a gRPC service written in C++ and deploy it to our Google Kubernetes Engine (GKE) cluster in the cloud in such a way that authentication is required for access. We will show how to access the service from the workstation and how to access it from code running in the robot's Kubernetes cluster.


## Prerequisites

* Completed the [Quickstart Guide](../quickstart.md), after which the GCP project is set up and `gcloud-sdk` and `kubectl` are installed and configured.
* `docker` is installed and configured on the workstation ([instructions](https://docs.docker.com/install/linux/docker-ce/ubuntu/)).
* `git` and `make` are installed on the workstation.
* To build and run locally: The gRPC C++ library and `protoc` are installed on the workstation ([instructions](https://github.com/grpc/grpc/blob/master/src/cpp/README.md)).
* For the last part of the guide: A robot that has been successfully [connected to the cloud](connecting-robot.md).

All files for this tutorial are located in
[docs/how-to/examples/greeter-service/](https://github.com/googlecloudrobotics/core/tree/master/docs/how-to/examples/greeter-service).

```
git clone https://github.com/googlecloudrobotics/core
cd core/docs/how-to/examples/greeter-service
```

Set your GCP project ID as an environment variable:

```
export PROJECT_ID=[YOUR_GCP_PROJECT_ID]
```


## Running gRPC server and client locally

<!--
TODO(skopecki) Replace this section by a version using Docker images for testing, so the user
    does not need to install the gRPC library.
-->

You may skip this section if you don't have `grpc` and `protoc` installed yet.

We will use [gRPC's quickstart example](https://grpc.io/docs/quickstart/cpp.html) with small modifications.
If you like to learn more about gRPC in C++, follow their guide first.

The gRPC `helloworld.Greeter` service is defined in `proto/helloworld.proto`.
It accepts a `HelloRequest` containing a `name` and responds with a `HelloReply` containing a `message`.
The server is implemented in `server/server.cc` and the client is implemented `client/client.cc`. The client sends the request with `name: "world"` to the server which responds with `message: "Hello <name>"`.

Build and run `greeter_server` locally:

```
make greeter-server
./greeter-server
```

In a different console build and run the client:

```
make greeter-client
./greeter-client localhost
```

You should see `Greeter received: Hello world` in the client's output and `Received request: name: "world"` in the server's output. You can also send your own name in the gRPC request to the server, try:

```
./greeter-client localhost $USER
```

Shut down the server with `Ctrl+C`.


## Dockerizing the service

To prepare our C++ program for deployment in the cloud, we package it as a Docker image.
Make sure the docker daemon is running and your user has the necessary privileges:

```
docker run --rm hello-world
```

If this command fails, make sure Docker is installed according to the [installation instructions](https://docs.docker.com/install/linux/docker-ce/ubuntu/).

The Docker image for the server is configured in the `server/Dockerfile`:

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

To build the Docker image, run:

```
docker build -t greeter-server -f server/Dockerfile .
```

> **Note**
> The docker file is in the subfolder `greeter-server/server/`, but the docker command must be called from `greeter-server/` to include the files which are shared between the client and the server.

You should now have an image tagged `greeter-server` in your local registry:

```
docker images | grep greeter-server
```

It can be run locally with:

```
docker run --name greeter-server -ti --rm -p 50051:50051 greeter-server
```

You should be able to send requests from the client as before:

```
./greeter-client localhost
```

If you are not able to stop the server using `Ctrl+C`, you can stop it from another terminal by running (this works because we named the image with `--name greeter-service`):

```
docker stop greeter-server
```

## Uploading the Docker image to the cloud

In order to be able to run the server as a container in our cloud cluster, we need to upload the Docker image to our GCP project's private [container registry](https://cloud.google.com/container-registry/docs/pushing-and-pulling).

Enable the Docker credential helper:

```
gcloud auth configure-docker
```

Tag the image and push it to the registry:

```
docker tag greeter-server gcr.io/$PROJECT_ID/greeter-server
docker push gcr.io/$PROJECT_ID/greeter-server
```

The image should now show up in the [container registry](https://console.cloud.google.com/gcr).


## Deploying the service in the cloud using Kubernetes

Run the following command to create `greeter-server.yaml` using the provided template:

```
cat greeter-server.yaml.tmpl | envsubst >greeter-server.yaml
```

This file contains the information needed by Kubernetes to run the gRPC service in our cloud cluster.
The three resources, Ingress, Service, and Deployment, are explained in the [deploying a service tutorial](deploying-service.md).
In contrast to the other tutorial, the Ingress tells nginx to forward incoming requests to a gRPC backend.

[embedmd]:# (examples/greeter-service/greeter-server.yaml.tmpl yaml /^/ /---/)
```yaml
apiVersion: extensions/v1beta1
kind: Ingress
metadata:
  name: greeter-server-ingress
  annotations:
    kubernetes.io/ingress.class: "nginx"
    nginx.ingress.kubernetes.io/grpc-backend: "true"
    nginx.ingress.kubernetes.io/secure-backends: "false"
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

```
kubectl config get-contexts
```

If the correct cluster is not marked with an asterisk in the output, you can switch contexts with `kubectl config use-context [...]`.)
Then deploy by applying the configuration:

```
kubectl apply -f greeter-server.yaml
```

You can explore the various resources that were created on your cluster as a result of this command in the [GKE Console](https://console.cloud.google.com/kubernetes/workload) or with `kubectl`, e.g.:

```
kubectl get pods
```

The resulting list should contain a running pod with a name like `greeter-server-xxxxxxxxxx-xxxxx`.


## Redeploying after a change

For convenience, `deploy.sh` provides some commands to create, delete, and update the service.
If you make changes to `greeter-server.yaml.tmpl`, all you have to do is run:

```
./deploy.sh update_config
```

If you make changes to `server.cc`, you need to run:

```
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


Let's try to access our server as before. We have to connect to the nginx ingress which is hosted on `www.endpoints.$PROJECT_ID.cloud.goog:443`:

```
./greeter-client www.endpoints.$PROJECT_ID.cloud.goog:443
```

> **Known issue**
> If this command fails to load your credentials file, try:
> ```
> export GRPC_DEFAULT_SSL_ROOTS_FILE_PATH=/etc/ssl/certs/ca-certificates.crt
> ```

Recall that when running `./greeter-server` on your workstation you were able to see the server's log output upon receiving a request.
This log output is also recorded when the server is running in the cloud cluster. To inspect it, run:

```
kubectl logs -l 'app=greeter-server-app'
```

Or go to the [GKE Console](https://console.cloud.google.com/kubernetes/workload), select the `greeter-server` workload and click on "Container logs".


## Accessing the API from the robot

In order to run `greeter-client` on the robot's Kubernetes cluster, we again package it as a Docker image and push it to our container registry, to which the robot also has access.
`client/Dockerfile` describes the client's image which is similar to the server's Dockerfile.

Our deploy script offers a command to build, tag, and push the image to the cloud registry, like we did with the server container:

```
./deploy.sh push_client
```

And finally, to execute the script, SSH into robot and run:

```
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

```
./deploy.sh delete
```
