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
    sha256 = "eea0f59c28a9241156a47d7a8e32db9122f3d50b505fae0f33de6ce4d9b61834",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/rules_pkg/releases/download/0.8.0/rules_pkg-0.8.0.tar.gz",
        "https://github.com/bazelbuild/rules_pkg/releases/download/0.8.0/rules_pkg-0.8.0.tar.gz",
    ],
)

load("@rules_pkg//:deps.bzl", "rules_pkg_dependencies")

rules_pkg_dependencies()

load("@io_bazel_rules_go//go:deps.bzl", "go_register_toolchains", "go_rules_dependencies")

# Do not call go_rules_dependencies() until after all other go_repository
# calls, or else the top-level definitions may be silently ignored.
# https://github.com/bazelbuild/bazel/issues/3908
go_register_toolchains(version = "1.19.2")

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

load(
    "@io_bazel_rules_docker//repositories:repositories.bzl",
    container_repositories = "repositories",
)

container_repositories()

load("@io_bazel_rules_docker//repositories:deps.bzl", container_deps = "deps")

container_deps()

load(
    "@io_bazel_rules_docker//container:container.bzl",
    "container_pull",
)

# When updating iptables_base, make sure to use the SHA of the amd64 image
# rather than the manifest list. You can do this by going to
# https://console.cloud.google.com/gcr/images/google-containers/GLOBAL/debian-iptables
# then selecting the latest version, expanding the manifest and finding the
# digest for the amd64 image.
# See https://github.com/bazelbuild/rules_docker/issues/714 for background.
container_pull(
    name = "iptables_base",
    digest = "sha256:9c41b4c326304b94eb96fdd2e181aa6e9995cc4642fcdfb570cedd73a419ba39",
    registry = "gcr.io",
    repository = "google-containers/debian-iptables",
)

load("@io_bazel_rules_docker//cc:image.bzl", _cc_image_repos = "repositories")

_cc_image_repos()

# Containerization rules for Go must come after go_rules_dependencies().
load("@io_bazel_rules_docker//go:image.bzl", _go_image_repos = "repositories")

_go_image_repos()

# grafana dashboards for nginx ingress controller

http_archive(
    name = "ingress-nginx",
    build_file = "//third_party:ingress-nginx.BUILD",
    sha256 = "719a7a54fe8156a38075eb99f82819b661ff117a2b043b41c1f560aaf71d4a09",
    strip_prefix = "ingress-nginx-1b1f7d30a39f71cd9fe9f7191258e983dcb159c6",
    urls = [
        "https://github.com/kubernetes/ingress-nginx/archive/1b1f7d30a39f71cd9fe9f7191258e983dcb159c6.tar.gz",
    ],
)
