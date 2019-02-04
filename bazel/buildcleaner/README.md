# buildcleaner

buildcleaner is a command line tool to apply automatic rewrites to
[Bazel](https://github.com/bazelbuild/bazel) BUILD and WORKSPACE files.

## Examples

Remove `--dry_run` to edit the files.

**WORKSPACE management**
```
bazel run -- bazel/buildcleaner fixWorkspace --workspace $PWD --dry_run
```

This will print `failed to mirror` errors that can be safely ignored.
These errors list the dependencies that are not in the bazel-mirror GCS bucket.
However, the future of bazel-mirror is uncertain, so keeping it up-to-date for
our build is not a priority.

**Dependency license checks**

Running `getLicenses` generates a list of licenses of dependencies.

Note: it only gets licenses for `http_archive` rules.

```
bazel run -- bazel/buildcleaner getLicenses --workspace $PWD | sort
```
