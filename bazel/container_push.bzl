load("@rules_oci//oci:defs.bzl", "oci_push")

def container_push(*args, **kwargs):
    """Creates a script to push a container image to a Docker registry. The
    target name must be specified when invoking the push script."""
    if "repository" in kwargs:
        fail(
            "Cannot set 'repository' attribute on container_push",
            attr = "repository",
        )
    kwargs["repository"] = "IGNORE"
    oci_push(*args, **kwargs)
