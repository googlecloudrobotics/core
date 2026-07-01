# Source Code

This directory contains the source code for Cloud Robotics Core components. All
components are written in `Go` and everything is build using `bazel`.

## Bazel

Run `bazel build //...` to build everything and likewise `bazel test //...` to
run all tests. For a test coverage report run:

```shell
bazel coverage --combined_report=lcov //...
genhtml --branch-coverage --output genhtml "$(bazel info output_path)/_coverage/_coverage_report.dat"
xdg-open genhtml/index.html
```

## Go

The [Gazelle](https://github.com/bazelbuild/bazel-gazelle) tool manages bazel
BUILD files for Go.

### Dependencies

To automatically update dependencies in bazel BUILD files run:

```shell
bazel run //:gazelle
```

To re-generate Go modules dependencies run this from the top-level source
directory:

```shell
./gomod.sh
```

This will always download the latest stable tag of a go module. To use a
specific version run eg::

```shell
cd src
# use an older version, that the latest stable
go get -d github.com/mitchellh/go-server-timing@v1.0.1
# use a specific, yet untagged version
go get -d github.com/mitchellh/go-server-timing@feb680ab92c20d57c527399b842e1941bde888c3
# to also upgrade dependencies, use:
go get -d -u github.com/mitchellh/go-server-timing@v1.0.1
```

More tips on this [one-pager](https://encore.dev/guide/go.mod)

### Licenses

Install go-license:

```shell
go install github.com/google/go-licenses@latest
```

run it:

```shell
cd src
# get a vsc of all licenses
~/go/bin/go-licenses csv .
# check for bad (forbidden) licenses, should be empty
~/go/bin/go-licenses check ./...
```

### Docs

In order to force a new snapshot, run

```shell
VERSION=$(curl -s https://proxy.golang.org/github.com/googlecloudrobotics/core/@latest | jq -r ".Version")
echo "https://pkg.go.dev/github.com/googlecloudrobotics/core/src/go@${VERSION}"
```

and open the printed link. Then that version is part of the history.

## third party

We track some external deps through [nvchecker](https://github.com/lilydjwg/nvchecker).
Get the tool by running:

```shell
pip3 install nvchecker
```

Below are sample commands for the common workflows. Run all those from the root
of the repo.

Add new dependency by adding a blob to nvchecker.toml:

```toml
[ingress-nginx]
source = "container"
registry = "k8s.gcr.io"
container = "ingress-nginx/controller"
prefix = "v"
```

Get initial version (use same command to update the version):

```shell
$ nvtake -c nvchecker.toml ingress-nginx=0.44.0
```

After updating, please also manually keep METADATA file in sync.

Check for updates:

```shell
$ nvchecker -c nvchecker.toml
[I 09-06 12:26:20.253 core:354] ingress-nginx: updated from 0.44.0 to 1.0.0
```

