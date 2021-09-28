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

To re-generate Go modules dependencies run this from the top-level source
directory:

```
./src/gomod.sh
```

This will always download the latest stable tag of a go module. To use a
specific version run eg::

```
cd src
# use an older version, that the latest stable
go get github.com/mitchellh/go-server-timing@v1.0.1
# use a specific, yet untagged version
go get github.com/mitchellh/go-server-timing@feb680ab92c20d57c527399b842e1941bde888c3
```

### Docs

In order to force a new snapshot, run
```bash
VERSION=$(curl -s https://proxy.golang.org/github.com/googlecloudrobotics/core/@latest | jq -r ".Version")
echo "https://pkg.go.dev/github.com/googlecloudrobotics/core/src/go@${VERSION}"
```
and open the printed link. Then that version is part of the history.

