# Testing instructions

Use `bazel run :push` on the app directory to build & upload Docker images,
upload Helm charts and update the app manifest:

```bash
bazel run //src/app_charts/ros:push
```

# Apps

## Mission CRD

The Mission CRD App creates the mission custom resource definition in the cloud and on the robot. The mission custom resources are used to send commands to the robot and are actuated by a robot-type-specific controller.
