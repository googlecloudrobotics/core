Google's Cloud Robotics Core is an open source platform that provides
infrastructure essential to building and running robotics solutions for business
automation. Cloud Robotics Core makes managing robot fleets easy for developers,
integrators, and operators. It enables:

* packaging and distribution of applications
* secure, bidirectional robot-cloud communication
* easy access to Google Cloud services such as ML, logging, and monitoring.

![Cloud Robotics Core overview](cloud-robotics-core-overview.png)

### Documentation

* [Quickstart](quickstart.md): Set up Cloud Robotics from binaries.
* [Overview](overview.md): Develop a deeper understanding of Cloud Robotics.
* Concepts
    * Layer 1: [Federation](concepts/federation.md)
    * Layer 2: [Application Management](concepts/app-management.md)
* How-to guides
    * [Deploying Cloud Robotics from sources](how-to/deploy-from-sources)<br/>
      Build and deploy Cloud Robotics from the sources hosted on Github using
      Bazel.
    * [Running a ROS node as a Kubernetes deployment](how-to/running-ros-node.md)<br/>
      Use Kubernetes to administer containerized workloads on a robot.
    * [Setting up OAuth for web UIs](how-to/setting-up-oauth.md)<br/>
      Use services like Grafana with a web browser.
    * [Connecting a robot to the cloud](how-to/connecting-robot.md)<br/>
      Enable secure communication between a robot and the Google Cloud Platform.
    * [Using Cloud Storage from a robot](how-to/using-cloud-storage.md)<br/>
      Programmatically store data from the robot with Cloud Storage.
    * [Deploying a service to the cloud](how-to/deploying-service.md)<br/>
      Run an API service in the cloud cluster and access it from a robot.
    * [Deploying a gRPC service](how-to/deploying-grpc-service.md)<br/>
      Run an gRPC service written in C++ in the cloud cluster and access it from a robot.
    * [Creating a declarative API](how-to/creating-declarative-api.md)<br/>
      Create a Kubernetes-style declarative API and run it on the cloud or on a robot.
* Development
    * [Debugging authentication problems](developers/debug-auth.md)<br/>
      Useful tips for working with Authentication and Authorization systems.
