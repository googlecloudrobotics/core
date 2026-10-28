load("@rules_img//img:push.bzl", "image_push")

def container_push(*args, **kwargs):
    """Creates an executable to push a container image to a Docker registry.
    The target registry and repository must be specified at runtime."""
    # Disallow setting 'registry' or 'repository' at build time so that the
    # target registry must be specified at runtime via '--registry' and
    # '--repository' flags.
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
    kwargs["registry"] = "unset"
    kwargs["repository"] = "unset"
    image_push(*args, **kwargs)
