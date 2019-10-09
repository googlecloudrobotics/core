# Source Code

This directory contains the source code for Cloud Robotics Core components. Most
components are written in Go and Java.

## Go

The [Gazelle](https://github.com/bazelbuild/bazel-gazelle) tool manages bazel
BUILD files for Go.

### Dependencies

To automatically update dependencies in bazel BUILD files run:

```
bazel run //:gazelle
```

To re-generate Go modules dependencies and add them to the WORKSPACE run this
from the top-level source directory:

```
./src/dep.sh ensure
```
