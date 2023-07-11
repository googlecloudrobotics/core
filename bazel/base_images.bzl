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
    # 3. Go to `<image>` in a browser and select the image you want. The digest will be listed along
    #    some other fields.
    #
    # Note that `oci_pull` supports tags if we ever want to use them.
    # Example: image = "gcr.io/distroless/base:latest"
    oci_pull(
        name = "go_image_base",
        digest = "sha256:df13a91fd415eb192a75e2ef7eacf3bb5877bb05ce93064b91b83feef5431f37",
        image = "gcr.io/distroless/base",
    )
    oci_pull(
        name = "go_debug_image_base",
        digest = "sha256:b57f14dff8aac3e09a24fa55d64fb6d36b95bdadead01a4c0b45e55d656e586b",
        image = "gcr.io/distroless/base",
    )
    oci_pull(
        name = "go_image_static",
        digest = "sha256:7198a357ff3a8ef750b041324873960cf2153c11cc50abb9d8d5f8bb089f6b4e",
        image = "gcr.io/distroless/static",
    )
    oci_pull(
        name = "go_debug_image_static",
        digest = "sha256:18740b995b4eac1b5706392a96ff8c4f30cefac18772058a71449692f1581f0f",
        image = "gcr.io/distroless/static",
    )
