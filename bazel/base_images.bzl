load("@io_bazel_rules_docker//repositories:go_repositories.bzl", _go_deps = "go_deps")
load("@rules_oci//oci:pull.bzl", "oci_pull")

def go_base_images():
    """
    This function pulls base images used to construct go application images with `rules_docker`.
    `rules_docker` provides its own `repositories` macro for the same reason, but it has a fixed version and they are
    no longer making versioned releases. The purpose of this macro is to allow us to update the base image version
    without having to update `rules_docker` version.
    """
    # This call is technically optional but is included here for safety as it is idempotent and must be run at least
    # once.
    _go_deps()

    # Images from the `gcr.io/distroless` repository are used. These are the same images used by `rules_docker`.
    #
    # In order to change the image versions you will need to update the digests in this file.
    # There are different ways you can get digests for images:
    #
    # 1. Run `docker pull <image>`, and copy the digest it returns.
    # 2. If you have downloaded the image, you can run
    #    `docker inspect <image> | grep RepoDigests -A2`
    #    and copy the digest.
    # 3. Go to `gcr.io/distroless/base` in a browser and select the image you want. The digest will be listed along
    #    some other fields.
    #
    # Note that `oci_pull` supports tags if we ever want to use them.
    # Example: image = "gcr.io/distroless/base:latest"
    oci_pull(
        name = "go_image_base",
        digest = "sha256:e711a716d8b7fe9c4f7bbf1477e8e6b451619fcae0bc94fdf6109d490bf6cea0",
        image = "gcr.io/distroless/base",
    )
    oci_pull(
        name = "go_debug_image_base",
        digest = "sha256:357bc96a42d8db2e4710d8ae6257da3a66b1243affc03932438710a53a8d1ac6",
        image = "gcr.io/distroless/base",
    )
    oci_pull(
        name = "go_image_static",
        digest = "sha256:e711a716d8b7fe9c4f7bbf1477e8e6b451619fcae0bc94fdf6109d490bf6cea0",
        image = "gcr.io/distroless/static",
    )
    oci_pull(
        name = "go_debug_image_static",
        digest = "sha256:357bc96a42d8db2e4710d8ae6257da3a66b1243affc03932438710a53a8d1ac6",
        image = "gcr.io/distroless/static",
    )
