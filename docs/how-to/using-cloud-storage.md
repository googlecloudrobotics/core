# Using Cloud Storage from a robot

Estimated time: 20 minutes

This page describes a simple Cloud Storage transaction that demonstrates how Google Cloud APIs can be accessed without additional authentication configuration from within the robot's Kubernetes cluster.

Normally, to access a private Cloud Storage bucket from a robot, you'd need to manage a service account for the robot through Identity & Access Management (IAM). Cloud Robotics handles the robot's identity for you, so you can connect securely without additional configuration.

1. If you haven't already, complete the [Connecting a robot to the cloud](connecting-robot.md) steps.

1. Choose a name for the Cloud Storage bucket.

    In the course of this guide, the robot will upload a file into a private bucket. The bucket namespace is global, so we must take care to choose a bucket name that is not in use yet by any other user of GCP. See also the [bucket naming requirements](https://cloud.google.com/storage/docs/naming), and [best practices](https://cloud.google.com/storage/docs/best-practices#naming).

    For this guide we will assume a bucket name like `robot-hello-world-dc1bb474`, where the part after the last dash is a random hexadecimal number. You can generate your own unique bucket name with the command

    ```shell
    echo robot-hello-world-$(tr -dc 'a-f0-9' < /dev/urandom | head -c8)
    ```

    Note: If the bucket name is already in use, creating the bucket in the next step will fail. In this case, choose a different bucket name.

1. Create the Cloud Storage bucket.

    On your workstation, run:

    ```shell
    gsutil mb gs://[BUCKET_NAME]
    ```

    Replace `[BUCKET_NAME]` with the name of the bucket you created, e.g., `robot-hello-world-dc1bb474`.
    `gsutil` is the command line tool for accessing Cloud Storage, it is part of the `gcloud-sdk` package; `mb` stands for "make bucket".

    Note that the bucket is not publicly writable, as can be verified in the [Cloud Storage browser](https://console.cloud.google.com/storage/browser).

1. Drop a file into the bucket from the robot.

    On the robot, run:

    ```console
    docker pull python:alpine
    kubectl run python --restart=Never --rm -ti --image=python:alpine -- /bin/sh
    # apk add gcc musl-dev libffi-dev
    # pip3 install google-cloud-storage
    # python3
    >>> from google.cloud import storage
    >>> client = storage.Client()
    >>> bucket = client.bucket("[BUCKET_NAME]")
    >>> bucket.blob("hello_world.txt").upload_from_string("Hello, I am a robot!\n")
    ```

    Replace `[BUCKET_NAME]` with the name of the bucket you created.

1. Verify that the file was uploaded.

    On your workstation, run:

    ```shell
    gsutil cat gs://[BUCKET_NAME]/hello_world.txt
    ```

    This should result in the output `Hello, I am a robot!`.

So why was the robot able to drop a file in the non-public bucket? There is a lot going on in the background that enabled the configuration-less secure API access:

* When the robot was connected to the cloud, it generated a new private key and registered the corresponding public key in a Cloud IoT device registry.
* The setup-robot command also started a Metadata Server as a workload in the robot's Kubernetes cluster. You can verify it is running with `kubectl get pods`. The Metadata Server identifies itself to the cloud using the robot's private key and obtains short-lived access tokens in the background.
* Every time a client library performs a call to a Google Cloud API, it asks the local Metadata Server for an access token.
* The permissions of the robot can be inspected and managed in the Cloud Console under "IAM &amp; admin"; you will notice that there is a service account called `robot-service@[PROJECT_ID].iam.gserviceaccount.com`, which has "Storage Admin" permissions. These permissions allowed the robot to write to the private bucket.

What's next:

* You can experiment with accessing other Google Cloud APIs, such as [Logging](https://cloud.google.com/logging/docs/) or [Pub/Sub](https://cloud.google.com/pubsub/docs/), from the robot programmatically. Also, Python is not the only programming language with Google Cloud client libraries: the APIs can be accessed, e.g., from code written in [Go](https://cloud.google.com/storage/docs/reference/libraries#client-libraries-install-go) in a similar configuration-less manner.
* [Write your own service](deploying-service.md) that runs as a container in the cloud and provides an API that can be accessed securely from the robot.
