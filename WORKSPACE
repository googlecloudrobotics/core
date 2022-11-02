workspace(name = "cloud_robotics")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")
load("//bazel:repositories.bzl", "cloud_robotics_repositories")

# gazelle:repo bazel_gazelle
cloud_robotics_repositories()

load("@com_google_protobuf//:protobuf_deps.bzl", "protobuf_deps")

protobuf_deps()

http_archive(
    name = "rules_pkg",
    sha256 = "8a298e832762eda1830597d64fe7db58178aa84cd5926d76d5b744d6558941c2",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/rules_pkg/releases/download/0.7.0/rules_pkg-0.7.0.tar.gz",
        "https://github.com/bazelbuild/rules_pkg/releases/download/0.7.0/rules_pkg-0.7.0.tar.gz",
    ],
)

load("@rules_pkg//:deps.bzl", "rules_pkg_dependencies")

rules_pkg_dependencies()

# Add Maven dependencies
load("//third_party:maven_dependencies.bzl", "maven_dependencies")

maven_dependencies()

load("@io_bazel_rules_go//go:deps.bzl", "go_register_toolchains")
load("@bazel_gazelle//:deps.bzl", "go_repository")

# Do not call go_rules_dependencies() until after all other go_repository
# calls, or else the top-level definitions may be silently ignored.
# https://github.com/bazelbuild/bazel/issues/3908
go_register_toolchains(version = "1.17.1")

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

load("@io_bazel_rules_go//go:deps.bzl", "go_rules_dependencies")

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

load(
    "@io_bazel_rules_docker//cc:image.bzl",
    _cc_image_repos = "repositories",
)

_cc_image_repos()

load(
    "@io_bazel_rules_docker//java:image.bzl",
    _java_image_repos = "repositories",
)

_java_image_repos()

# Containerization rules for Go must come after go_rules_dependencies().
load(
    "@io_bazel_rules_docker//go:image.bzl",
    _go_image_repos = "repositories",
)

_go_image_repos()

go_repository(
    name = "grpc_ecosystem_grpc_gateway",
    commit = "50c55a9810a974dc5a9e7dd1a5c0d295d525f283",
    importpath = "github.com/grpc-ecosystem/grpc-gateway",
)

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
