workspace(name = "cloud_robotics")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("//bazel:repositories.bzl", "cloud_robotics_repositories")

BAZEL_TOOLCHAIN_TAG = "0.8.1"

BAZEL_TOOLCHAIN_SHA = "751bbe30bcaa462aef792b18bbd16c401af42fc937c42ad0ae463f099dc04ea2"

http_archive(
    name = "com_grail_bazel_toolchain",
    canonical_id = BAZEL_TOOLCHAIN_TAG,
    sha256 = BAZEL_TOOLCHAIN_SHA,
    strip_prefix = "bazel-toolchain-{tag}".format(tag = BAZEL_TOOLCHAIN_TAG),
    url = "https://github.com/grailbio/bazel-toolchain/archive/{tag}.tar.gz".format(tag = BAZEL_TOOLCHAIN_TAG),
)

# Sysroot and libc
# How to upgrade:
# - Find image in https://storage.googleapis.com/chrome-linux-sysroot/ for amd64 for
#   a stable Linux (here: Debian stretch), of this pick a current build.
# - Verify the image contains expected /lib/x86_64-linux-gnu/libc* and defines correct
#   __GLIBC_MINOR__ in /usr/include/features.h
# - If system files are not found, add them in ../BUILD.sysroot
http_archive(
    name = "com_googleapis_storage_chrome_linux_amd64_sysroot",
    build_file = "//bazel:BUILD.sysroot",
    sha256 = "66bed6fb2617a50227a824b1f9cfaf0a031ce33e6635edaa2148f71a5d50be48",
    urls = [
        # features.h defines GLIBC 2.24. Contains /lib/x86_64-linux-gnu/libc-2.24.so,
        # last modified by Chrome 2018-02-22.
        "https://storage.googleapis.com/chrome-linux-sysroot/toolchain/15b7efb900d75f7316c6e713e80f87b9904791b1/debian_stretch_amd64_sysroot.tar.xz",
    ],
)

load("@com_grail_bazel_toolchain//toolchain:deps.bzl", "bazel_toolchain_dependencies")

bazel_toolchain_dependencies()

load("@com_grail_bazel_toolchain//toolchain:rules.bzl", "llvm_toolchain")

# How to upgrade:
# - Pick a new version that runs on a stable OS similar enough to our sysroot from
#   https://releases.llvm.org/download.html
# - Documentation is in
#   https://github.com/grailbio/bazel-toolchain/blob/master/toolchain/rules.bzl
# - If system files are not found, add them in bazel/BUILD.sysroot
llvm_toolchain(
    name = "llvm_toolchain",
    distribution = "clang+llvm-14.0.0-x86_64-linux-gnu-ubuntu-18.04.tar.xz",
    llvm_version = "14.0.0",
    sysroot = {
        "linux-x86_64": "@com_googleapis_storage_chrome_linux_amd64_sysroot//:all_files",
    },
)

load("@llvm_toolchain//:toolchains.bzl", "llvm_register_toolchains")

llvm_register_toolchains()

# gazelle:repo bazel_gazelle
cloud_robotics_repositories()

load("@com_google_protobuf//:protobuf_deps.bzl", "protobuf_deps")

protobuf_deps()

http_archive(
    name = "rules_pkg",
    sha256 = "8f9ee2dc10c1ae514ee599a8b42ed99fa262b757058f65ad3c384289ff70c4b8",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/rules_pkg/releases/download/0.9.1/rules_pkg-0.9.1.tar.gz",
        "https://github.com/bazelbuild/rules_pkg/releases/download/0.9.1/rules_pkg-0.9.1.tar.gz",
    ],
)

load("@rules_pkg//:deps.bzl", "rules_pkg_dependencies")

rules_pkg_dependencies()

load("@io_bazel_rules_go//go:deps.bzl", "go_register_toolchains", "go_rules_dependencies")

# Do not call go_rules_dependencies() until after all other go_repository
# calls, or else the top-level definitions may be silently ignored.
# https://github.com/bazelbuild/bazel/issues/3908
go_register_toolchains(version = "1.21.1")

http_archive(
    name = "com_github_kubernetes_sigs_application",
    build_file = "@cloud_robotics//third_party:app_crd.BUILD",
    sha256 = "8bafd7fb97563d1a15d9afc68c87e3aabd664f60bd8005f1ae685d79842c1ac4",
    strip_prefix = "application-c8e2959e57a02b3877b394984a288f9178977d8b",
    urls = [
        "https://github.com/kubernetes-sigs/application/archive/c8e2959e57a02b3877b394984a288f9178977d8b.tar.gz",
    ],
)

bazel_gomock_commit = "fde78c91cf1783cc1e33ba278922ba67a6ee2a84"

http_archive(
    name = "bazel_gomock",
    sha256 = "692421b0c5e04ae4bc0bfff42fb1ce8671fe68daee2b8d8ea94657bb1fcddc0a",
    strip_prefix = "bazel_gomock-{v}".format(v = bazel_gomock_commit),
    urls = [
        "https://github.com/jmhodges/bazel_gomock/archive/{v}.tar.gz".format(v = bazel_gomock_commit),
    ],
)

# Add Go dependencies
load("//third_party:go_repositories.bzl", "go_repositories")

# gazelle:repository_macro third_party/go_repositories.bzl%go_repositories
go_repositories()

go_rules_dependencies()

load("@bazel_gazelle//:deps.bzl", "gazelle_dependencies")

gazelle_dependencies()

http_archive(
    name = "rules_oci",
    sha256 = "176e601d21d1151efd88b6b027a24e782493c5d623d8c6211c7767f306d655c8",
    strip_prefix = "rules_oci-1.2.0",
    url = "https://github.com/bazel-contrib/rules_oci/releases/download/v1.2.0/rules_oci-v1.2.0.tar.gz",
)

# oci_rules is configured to pull rules_docker go base images
# The configuration was copied from the release documentation at
# https://github.com/bazel-contrib/rules_oci/releases/tag/v0.3.9 and then slightly
# modified to remove duplicate calls already present in this file.
load("@rules_oci//oci:dependencies.bzl", "rules_oci_dependencies")

rules_oci_dependencies()

load("@rules_oci//oci:repositories.bzl", "LATEST_CRANE_VERSION", "LATEST_ZOT_VERSION", "oci_register_toolchains")

oci_register_toolchains(
    name = "oci",
    crane_version = LATEST_CRANE_VERSION,
)

# grafana dashboards for nginx ingress controller

http_archive(
    name = "ingress-nginx",
    build_file = "//third_party:ingress-nginx.BUILD",
    sha256 = "6e571764828b24545eea49582fd56d66d51fc66e52a375d98251c80c57fdb2fc",
    strip_prefix = "ingress-nginx-controller-v1.8.0",
    urls = [
        "https://github.com/kubernetes/ingress-nginx/archive/refs/tags/controller-v1.8.0.tar.gz",
    ],
)

load("@rules_oci//oci:pull.bzl", "oci_pull")

# https://console.cloud.google.com/gcr/images/distroless/global/cc
oci_pull(
    name = "distroless_cc",
    digest = "sha256:b82f113425c5b5c714151aaacd8039bc141821cdcd3c65202d42bdf9c43ae60b",
    image = "gcr.io/distroless/cc",
    platforms = [
        "linux/amd64",
        "linux/arm64",
    ],
)

# https://console.cloud.google.com/gcr/images/distroless/global/base
oci_pull(
    name = "distroless_base",
    digest = "sha256:b31a6e02605827e77b7ebb82a0ac9669ec51091edd62c2c076175e05556f4ab9",
    image = "gcr.io/distroless/base",
    platforms = [
        "linux/amd64",
        "linux/arm64",
    ],
)

# When updating iptables_base, make sure to use the SHA of the amd64 image
# rather than the manifest list. You can do this by going to
# https://console.cloud.google.com/gcr/images/google-containers/GLOBAL/debian-iptables
# then selecting the latest version, expanding the manifest and finding the
# digest for the amd64 image.
# See https://github.com/bazelbuild/rules_docker/issues/714 for background.
oci_pull(
    name = "iptables_base",
    digest = "sha256:9c41b4c326304b94eb96fdd2e181aa6e9995cc4642fcdfb570cedd73a419ba39",
    image = "gcr.io/google-containers/debian-iptables",
)

load("@aspect_bazel_lib//lib:repositories.bzl", "register_jq_toolchains")

register_jq_toolchains()
