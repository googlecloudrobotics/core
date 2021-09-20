# Source Code

This directory contains the source code for Cloud Robotics Core components. Most
components are written in Go and Java.

## Go

The [Gazelle](https://github.com/bazelbuild/bazel-gazelle) tool manages bazel
BUILD files for Go.

### Dependencies

To automatically update dependencies in bazel BUILD files run:

```
go get github.com/bazelbuild/bazel-gazelle/cmd/gazelle_dependencies
gazelle
# https://github.com/bazelbuild/bazel-gazelle/issues/1076
# bazel run //:gazelle
```

To re-generate Go modules dependencies and add them to the WORKSPACE run this
from the top-level source directory:

```
./src/dep.sh ensure
```

### Docs

In order to force a new snapshot, run
```bash
VERSION=$(curl -s https://proxy.golang.org/github.com/googlecloudrobotics/core/@latest | jq -r ".Version")
echo "https://pkg.go.dev/github.com/googlecloudrobotics/core/src/go@${VERSION}"
```
and open the printed link. Then that version is part of the history.

