# Deploying a service to the cloud cluster

Estimated time: 60 min

In this guide we will write a HTTP service in Python and deploy it to our Google Kubernetes Engine (GKE) cluster in the cloud in such a way that authentication is required for access. We will show how to access the service from the workstation and how to access it from code running in the robot's Kubernetes cluster.

## Prerequisites

* Completed the [Quickstart Guide](../quickstart.md), after which the GCP project is set up and `gcloud-sdk` and `kubectl` are installed and configured.
* `docker` is installed and configured on the workstation ([instructions](https://docs.docker.com/install/linux/docker-ce/ubuntu/)).
* `python3`, `python3-pip`, and `curl` are installed on the workstation.
* For the last part of the guide: A robot that has been successfully [connected to the cloud](connecting-robot.md).

Create a directory for the code examples of this guide, e.g.:

```shell
mkdir hello-service
cd hello-service
```

Set your GCP project ID as an environment variable:

```shell
export PROJECT_ID=[YOUR_GCP_PROJECT_ID]
```

All files created in this tutorial can be found in
[docs/how-to/examples/hello-service/](https://github.com/googlecloudrobotics/core/tree/master/docs/how-to/examples/hello-service).
If you download the files, you have to replace the placeholders `[PROJECT_ID]` with your GCP project ID:

```shell
sed -i "s/\[PROJECT_ID\]/$PROJECT_ID/g" client/client.py server/hello-server.yaml
```

## A simple HTTP server

Create a subdirectory for the server code:

```shell
mkdir server
cd server
```

Paste the following into a file called `server.py`:

[embedmd]:# (examples/hello-service/server/server.py python)
```python
from http import server
import signal
import sys


class MyRequestHandler(server.BaseHTTPRequestHandler):
  def do_GET(self):
    print('Received a request')
    self.send_response(200)
    self.send_header('Content-Type', 'text/plain')
    self.end_headers()
    self.wfile.write(b'Server says hello!\n')


def main():
  # Terminate process when Kubernetes sends SIGTERM.
  signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))

  server_address = ('', 8000)
  httpd = server.HTTPServer(server_address, MyRequestHandler)
  httpd.serve_forever()


if __name__ == '__main__':
  main()
```

This Python program implements a server that listens on port 8000 for incoming HTTP GET requests. When such a request is received, it prints a line to stdout and responds to the request with a short message.

You can try it out with:

```shell
python server.py
```

If you see `ImportError: No module named http`, you are most likely using Python 2.x; try `python3` instead of `python`.)

Then, from another terminal on the same workstation, run:

```shell
curl -i http://localhost:8000
```

You should see the headers indicating that the request was successful (`200 OK`) and the server's response message. You can also try entering `localhost:8000` in your browser's address bar.

## Dockerizing the service

Next, to prepare our Python program for deployment in the cloud, we package it as a Docker image.

Make sure that the docker daemon is running and that your user has the necessary privileges:

```shell
docker run --rm hello-world
```

If this command fails, make sure Docker is installed according to the [installation instructions](https://docs.docker.com/install/linux/docker-ce/ubuntu/).

In the same directory as `server.py`, create a `Dockerfile` with the following contents:

[embedmd]:# (examples/hello-service/server/Dockerfile dockerfile)
```dockerfile
FROM python:alpine

WORKDIR /data

COPY server.py ./

CMD [ "python", "-u", "./server.py" ]
```

(Note: the `-u` option disables line-buffering; Python's line-buffering can prevent output from appearing immediately in the Docker logs.)

To build the Docker image, run:

```shell
docker build -t hello-server .
```

You should now have an image tagged `hello-server` in your local registry:

```shell
docker images | grep hello-server
```

It can be run locally with:

```shell
docker run -ti --rm -p 8000:8000 hello-server
```

You should now be able to send requests to the server with `curl` as before.

## Uploading the Docker image to the cloud

In order to be able to run the server as a container in our cloud cluster, we need to upload the Docker image to our GCP project's private [container registry](https://cloud.google.com/container-registry/docs/pushing-and-pulling).

Enable the Docker credential helper:

```shell
gcloud auth configure-docker
```

Tag the image and push it to the registry:

```shell
docker tag hello-server gcr.io/$PROJECT_ID/hello-server
docker push gcr.io/$PROJECT_ID/hello-server
```

The image should now show up in the [Container Registry](https://console.cloud.google.com/gcr).

## Deploying the service in the cloud using Kubernetes

Create a file called `hello-server.yaml` with the following contents:

[embedmd]:# (examples/hello-service/server/hello-server.yaml yaml)
```yaml
apiVersion: networking.k8s.io/v1beta1
kind: Ingress
metadata:
  name: hello-server-ingress
  annotations:
    kubernetes.io/ingress.class: "nginx"
    nginx.ingress.kubernetes.io/auth-url: "http://token-vendor.default.svc.cluster.local/apis/core.token-vendor/v1/token.verify?robots=true"
spec:
  rules:
  - host: www.endpoints.[PROJECT_ID].cloud.goog
    http:
      paths:
      - path: /apis/hello-server
        backend:
          serviceName: hello-server-service
          servicePort: 8000
---
apiVersion: v1
kind: Service
metadata:
  name: hello-server-service
spec:
  ports:
  - name: hello-server-port
    port: 8000
  # the selector is used to link pods to services
  selector:
    app: hello-server-app
---
apiVersion: apps/v1
kind: Deployment
metadata:
  name: hello-server
spec:
  # all pods matching this selector belong to this deployment
  selector:
    matchLabels:
      app: hello-server-app
  template:
    metadata:
      # the other side of the link between services and pods
      labels:
        app: hello-server-app
    spec:
      containers:
      - name: hello-server
        image: gcr.io/[PROJECT_ID]/hello-server:latest
        ports:
        # must match the port of the service
        - containerPort: 8000
```

This file contains the information needed by Kubernetes to run our HTTP service in our cloud cluster. In the following, we will go over it bit by bit assuming basic familiarity with the [YAML format](https://en.wikipedia.org/wiki/YAML).

We define three Kubernetes resources:

* The *Ingress* contains rules that tell our cluster's nginx (the HTTP server that handles all incoming traffic) which incoming requests to forward to our service.
* The *Service* defines how our service is exposed within the cluster.
* The *Deployment* describes the Docker container to run.

Metadata, labels, and selectors are used to tie the three resources together.

A detailed explanation of the Kubernetes resources is out of scope for this guide, check out the [Kubernetes docs](https://kubernetes.io/docs/home/) to get started. There are a few points worth mentioning, though:

* In the Ingress and Deployment, don't forget to replace `[PROJECT_ID]` with your GCP project ID.
* In the Ingress, there is an annotation with key `nginx.ingress.kubernetes.io/auth-url`. This tells our cluster's nginx to check the authorization of each request before forwarding it to the `hello-server`. The value `http://token-vendor...` is the cluster-internal DNS address of a token verifier service that is running in the cluster as part of the Cloud Robotics Core platform.
* The Ingress rules specify that our hello-server will be reachable at `https://www.endpoints.[PROJECT_ID].cloud.goog/apis/hello-server`.
* The Deployment contains the full reference of the Docker image that we pushed to the Container Registry in the previous section.

Make sure that `kubectl` points to the correct GCP project:

```shell
kubectl config get-contexts
```

If the correct cluster is not marked with an asterisk in the output, you can switch to it with `kubectl config use-context [...]`.)
Then deploy by applying the configuration:

```shell
kubectl apply -f hello-server.yaml
```

You can explore the various resources that were created on your cluster as a result of this command in the [GKE Console](https://console.cloud.google.com/kubernetes/workload) or with `kubectl`, e.g.:

```shell
kubectl get pods
```

The resulting list should contain a running pod with a name like `hello-server-xxxxxxxxxx-xxxxx`.

## Redeploying after a change

If you make a change to `server.py`, you need to rebuild and push the Docker image:

```shell
docker build -t hello-server .
docker tag hello-server gcr.io/$PROJECT_ID/hello-server
docker push gcr.io/$PROJECT_ID/hello-server
```

The easiest way to get Kubernetes to restart the workload with the latest version of the container is to delete the pod:

```shell
kubectl delete pod -l 'app=hello-server-app'
```

Kubernetes will automatically pull the newest image and recreate the pod.

If you make a change to `hello-server.yaml`, all you have to do is apply it again:

```shell
kubectl apply -f hello-server.yaml
```

## Accessing the API

Let's try to access our server as we did before:

```shell
curl -i https://www.endpoints.$PROJECT_ID.cloud.goog/apis/hello-server
```

This should result in a `401 Unauthorized` error because we did not supply any authorization information with the request.
(Note: If you comment out the `auth-url` annotation in the Ingress definition and reapply it, this request will succeed.)

We can, however, easily obtain credentials from `gcloud` and attach them to our request by means of an "Authorization" header:

```shell
token=$(gcloud auth application-default print-access-token)
curl -i -H "Authorization: Bearer $token" https://www.endpoints.$PROJECT_ID.cloud.goog/apis/hello-server
```

If this command fails because "Application Default Credentials are not available", you need to first run:

```shell
gcloud auth application-default login
```

And follow the instructions in your browser.

Recall that when running `server.py` on your workstation you were able to see the server's log output upon receiving a request. This log output is also recorded when the server is running in the cloud cluster. To inspect it, run:

```shell
kubectl logs -l 'app=hello-server-app'
```

Or go to the [GKE Console](https://console.cloud.google.com/kubernetes/workload), select the `hello-server` workload and click on "Container logs".

Next, let's access the API from some Python code. Eventually, we will build another Docker image from this code, so it needs to live in a separate directory:

```shell
cd ..
mkdir client
cd client
```

Get some dependencies:

```shell
pip install --upgrade google-auth requests
```

(Depending on your local installation, you might have to use `pip3`.)

Create `client.py` with the following contents:

[embedmd]:# (examples/hello-service/client/client.py python)
```python
import google.auth
import google.auth.transport.requests as requests

credentials, project_id = google.auth.default()

authed_session = requests.AuthorizedSession(credentials)

response = authed_session.request(
  "GET", "https://www.endpoints.[PROJECT_ID].cloud.goog/apis/hello-server")

print(response.status_code, response.reason, response.text)
```

Replace `[PROJECT_ID]` with your GCP project ID.

This script:

* uses [`google-auth`](https://google-auth.readthedocs.io/en/latest/user-guide.html) to obtain application default credentials (just as we previously did with the `gcloud` CLI),
* uses the [`requests`](http://docs.python-requests.org/en/stable/) library to perform an authenticated request to our API,
* prints the response to stdout.

Try it out:

```shell
python client.py
```

You will get a warning about using end user credentials. You can safely ignore this warning; we will eventually be using a robot's credentials.)

## Accessing the API from the robot

In order to run this script on the robot's Kubernetes cluster, we again package it as a Docker image and push it to our container registry, to which the robot also has access.

Create a `Dockerfile` containing:

[embedmd]:# (examples/hello-service/client/Dockerfile dockerfile)
```dockerfile
FROM python:alpine

RUN pip install --no-cache-dir google-auth requests

WORKDIR /data

COPY client.py ./

CMD [ "python", "-u", "./client.py" ]
```

Build, tag, and push the image:

```shell
docker build -t hello-client .
docker tag hello-client gcr.io/$PROJECT_ID/hello-client
docker push gcr.io/$PROJECT_ID/hello-client
```

And finally, to execute the script, SSH into robot and run:

```shell
kubectl run -ti --rm --restart=Never --image=gcr.io/$PROJECT_ID/hello-client hello-client
```

You should see the server's message.

Two things are noteworthy:

* The `hello-client` Docker image was pulled from the Container Registry without the need for additional credentials. This worked because there is a periodical job running on the robot's Kubernetes cluster that refreshes the GCR credentials. Run `kubectl get pods` on the robot and you will see pod names that start with `gcr-credential-refresher`.
* The `google.auth.default()` invocation in the Python code automatically obtained credentials that allowed the robot to access the cloud cluster. This worked because the `google-auth` library queried the local Metadata Server, which obtains access tokens for the robot in the background.

## Cleaning up

In order to stop the service in the cloud cluster and revert the configuration changes, change to the `server` directory and run:

```shell
kubectl delete -f hello-server.yaml
```

<!--
## What's next

There are a few inconvenient steps in this guide, e.g.:

* manually replacing the project ID in all source files or
* remotely logging in to the robot to start a workload.

This is where the app management layer comes in; it provides, among other capabilities:

* ways of bundling and parameterizing Kubernetes resources and
* remote management of Kubernetes resources/workloads on the robot.

Also note that both the server and the client side can be implemented with similar ease in other programming languages, such as [Java](https://developers.google.com/api-client-library/java/) or [Go](https://github.com/googleapis/google-api-go-client).
-->
