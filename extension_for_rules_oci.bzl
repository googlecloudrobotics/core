load("@rules_oci//oci/private:pull.bzl", "oci_alias", "oci_pull")

def _extension_for_rules_oci_impl(ctx):
    oci_alias(
        name = "distroless_base",
        scheme = "https",
        registry = "gcr.io",
        repository = "distroless/base",
        identifier = "sha256:b31a6e02605827e77b7ebb82a0ac9669ec51091edd62c2c076175e05556f4ab9",
        platforms = {
            "@platforms//cpu:x86_64": "@distroless_base_linux_amd64",
            "@platforms//cpu:arm64": "@distroless_base_linux_arm64",
        },
        target_name = "distroless_base",
    )
    oci_pull(
        name = "distroless_base_linux_amd64",
        scheme = "https",
        registry = "gcr.io",
        repository = "distroless/base",
        identifier = "sha256:b31a6e02605827e77b7ebb82a0ac9669ec51091edd62c2c076175e05556f4ab9",
        platform = "linux/amd64",
        target_name = "distroless_base_linux_amd64",
    )
    oci_alias(
        name = "distroless_cc",
        scheme = "https",
        registry = "gcr.io",
        repository = "distroless/cc",
        identifier = "sha256:b82f113425c5b5c714151aaacd8039bc141821cdcd3c65202d42bdf9c43ae60b",
        platforms = {
            "@platforms//cpu:x86_64": "@distroless_cc_linux_amd64",
            "@platforms//cpu:arm64": "@distroless_cc_linux_arm64",
        },
        target_name = "distroless_cc",
    )
    oci_pull(
        name = "distroless_cc_linux_amd64",
        scheme = "https",
        registry = "gcr.io",
        repository = "distroless/cc",
        identifier = "sha256:b82f113425c5b5c714151aaacd8039bc141821cdcd3c65202d42bdf9c43ae60b",
        platform = "linux/amd64",
        target_name = "distroless_cc_linux_amd64",
    )
    oci_alias(
        name = "iptables_base",
        scheme = "https",
        registry = "gcr.io",
        repository = "google-containers/debian-iptables",
        identifier = "sha256:9c41b4c326304b94eb96fdd2e181aa6e9995cc4642fcdfb570cedd73a419ba39",
        platform = "//external:iptables_base_single",
        target_name = "iptables_base",
    )
    oci_pull(
        name = "iptables_base_single",
        scheme = "https",
        registry = "gcr.io",
        repository = "google-containers/debian-iptables",
        identifier = "sha256:9c41b4c326304b94eb96fdd2e181aa6e9995cc4642fcdfb570cedd73a419ba39",
        target_name = "iptables_base_single",
    )

# -- repo definitions -- #

extension_for_rules_oci = module_extension(implementation = _extension_for_rules_oci_impl)
