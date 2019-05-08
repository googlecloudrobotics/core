load("@io_bazel_rules_docker//container:container.bzl", _container_push = "container_push")

def container_push(*args, **kwargs):
    """Creates a script to push a container image to a Docker registry. The
    target name must be specified when invoking the push script."""
    if "registry" in kwargs:
        fail(
            "Cannot set 'registry' attribute on container_push",
            attr = "registry",
        )
    if "repository" in kwargs:
        fail(
            "Cannot set 'repository' attribute on container_push",
            attr = "repository",
        )
    kwargs["registry"] = "IGNORE"
    kwargs["repository"] = "IGNORE"
    _container_push(*args, **kwargs)
