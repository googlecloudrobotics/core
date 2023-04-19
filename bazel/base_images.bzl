load("@io_bazel_rules_docker//repositories:go_repositories.bzl", _go_deps = "go_deps")
load("@rules_oci//oci:pull.bzl", "oci_pull")

def go_base_images():
    # This call is technically optional but is included here for safety as it is idempotent and required,
    _go_deps()

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
