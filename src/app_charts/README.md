# Testing instructions

Use `bazel run :push` on the app directory to build & upload Docker images,
upload Helm charts and update the app manifest:

```bash
bazel run //src/app_charts/ros:push
```
