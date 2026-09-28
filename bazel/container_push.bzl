load("@rules_oci//oci:defs.bzl", "oci_push")

def container_push(*args, **kwargs):
    """Creates a script to push a container image to a Docker registry. The
    target name must be specified when invoking the push script."""
    # Disallow setting 'repository' at build time so that the target registry
    # must be specified at runtime via the '--repository' flag (supported
    # natively without a dummy placeholder since rules_oci 2.2.7+).
    if "repository" in kwargs:
        fail(
            "Cannot set 'repository' attribute on container_push",
            attr = "repository",
        )
    oci_push(*args, **kwargs)
