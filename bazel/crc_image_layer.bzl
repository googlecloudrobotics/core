load("@rules_pkg//pkg:tar.bzl", "pkg_tar")

def crc_image_layer(**kwargs):
    """Creates a compressed tarball for an OCI container image layer."""
    # Use the hermetic C zstd binary in gzip compatibility mode instead of
    # rules_pkg's single-threaded Python gzip implementation. This cuts
    # packaging time in half while remaining backwards-compatible with older
    # containerd versions (< 1.5) and Docker v2 Schema 2 base images.
    kwargs.setdefault("compressor", "//bazel:zstd")
    kwargs.setdefault("compressor_args", "--format=gzip -6 -q")
    kwargs.setdefault("extension", "tar.gz")

    # Once all target deployments run containerd >= 1.5 (and use OCI manifests),
    # switch to native multithreaded zstd compression (~15x faster than Python
    # gzip and ~6% smaller image layers):
    # kwargs.setdefault("compressor", "//bazel:zstd")
    # kwargs.setdefault("compressor_args", "-3 -q")
    # kwargs.setdefault("extension", "tar.zst")
    #
    # Or for local-only development builds where registry push size does not
    # matter, use uncompressed tarballs:
    # kwargs.setdefault("extension", "tar")

    pkg_tar(**kwargs)
