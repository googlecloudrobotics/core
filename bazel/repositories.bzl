load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def cloud_robotics_repositories():
    _maybe(
        http_archive,
        name = "com_google_protobuf",
        sha256 = "f66073dee0bc159157b0bd7f502d7d1ee0bc76b3c1eac9836927511bdc4b3fc1",
        strip_prefix = "protobuf-3.21.9",
        urls = [
            "https://mirror.bazel.build/github.com/protocolbuffers/protobuf/archive/v3.21.9.zip",
            "https://github.com/protocolbuffers/protobuf/archive/v3.21.9.zip",
        ],
    )

    # Binary build of the helm package manager for Kubernetes
    _maybe(
        http_archive,
        name = "kubernetes_helm",
        build_file = "@cloud_robotics//third_party/helm2:BUILD.bazel",
        sha256 = "f3bec3c7c55f6a9eb9e6586b8c503f370af92fe987fcbf741f37707606d70296",
        strip_prefix = "linux-amd64",
        urls = [
            "https://get.helm.sh/helm-v2.17.0-linux-amd64.tar.gz",
        ],
    )
    _maybe(
        http_archive,
        name = "kubernetes_helm3",
        build_file = "@cloud_robotics//third_party/helm3:BUILD.bazel",
        sha256 = "1484ffb0c7a608d8069470f48b88d729e88c41a1b6602f145231e8ea7b43b50a",
        strip_prefix = "linux-amd64",
        urls = [
            "https://get.helm.sh/helm-v3.9.0-linux-amd64.tar.gz",
        ],
    )

    # Binary build of the terraform infrastructure management tool
    _maybe(
        http_archive,
        name = "hashicorp_terraform",
        build_file = "@cloud_robotics//third_party:terraform.BUILD",
        sha256 = "e5eeba803bc7d8d0cae7ef04ba7c3541c0abd8f9e934a5e3297bf738b31c5c6d",
        urls = [
            "https://releases.hashicorp.com/terraform/0.12.31/terraform_0.12.31_linux_amd64.zip",
        ],
    )

    # Go rules and proto support
    _maybe(
        http_archive,
        name = "io_bazel_rules_go",
        sha256 = "91585017debb61982f7054c9688857a2ad1fd823fc3f9cb05048b0025c47d023",
        urls = [
            "https://mirror.bazel.build/github.com/bazelbuild/rules_go/releases/download/v0.42.0/rules_go-v0.42.0.zip",
            "https://github.com/bazelbuild/rules_go/releases/download/v0.42.0/rules_go-v0.42.0.zip",
        ],
    )
    _maybe(
        http_archive,
        name = "bazel_gazelle",
        sha256 = "b7387f72efb59f876e4daae42f1d3912d0d45563eac7cb23d1de0b094ab588cf",
        urls = [
            "https://mirror.bazel.build/github.com/bazelbuild/bazel-gazelle/releases/download/v0.34.0/bazel-gazelle-v0.34.0.tar.gz",
            "https://github.com/bazelbuild/bazel-gazelle/releases/download/v0.34.0/bazel-gazelle-v0.34.0.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_bazelbuild_buildtools",
        sha256 = "05c3c3602d25aeda1e9dbc91d3b66e624c1f9fdadf273e5480b489e744ca7269",
        strip_prefix = "buildtools-6.4.0",
        urls = ["https://github.com/bazelbuild/buildtools/archive/v6.4.0.tar.gz"],
    )

def _maybe(repo_rule, name, **kwargs):
    """
    Runs a named rule if a target with the rule name hasn't already been defined.
    """
    if name not in native.existing_rules():
        repo_rule(name = name, **kwargs)
