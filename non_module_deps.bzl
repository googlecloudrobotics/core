load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
# -- load statements -- #

def _non_module_deps_impl(ctx):
    # Sysroot and libc
    # How to upgrade:
    # - Find image in https://storage.googleapis.com/chrome-linux-sysroot/ for amd64 for
    #   a stable Linux (here: Debian bullseye), of this pick a current build.
    # - Verify the image contains expected /lib/x86_64-linux-gnu/libc* and defines correct
    #   __GLIBC_MINOR__ in /usr/include/features.h
    # - If system files are not found, add them in ../BUILD.sysroot
    http_archive(
        name = "com_googleapis_storage_chrome_linux_amd64_sysroot",
        build_file = Label("//bazel:BUILD.sysroot"),
        sha256 = "5df5be9357b425cdd70d92d4697d07e7d55d7a923f037c22dc80a78e85842d2c",
        urls = [
            # features.h defines GLIBC 2.31.
            "https://storage.googleapis.com/chrome-linux-sysroot/toolchain/4f611ec025be98214164d4bf9fbe8843f58533f7/debian_bullseye_amd64_sysroot.tar.xz",
        ],
    )
    http_archive(
        name = "bazel_gomock",
        urls = [
            "https://github.com/jmhodges/bazel_gomock/archive/fde78c91cf1783cc1e33ba278922ba67a6ee2a84.tar.gz",
        ],
        sha256 = "692421b0c5e04ae4bc0bfff42fb1ce8671fe68daee2b8d8ea94657bb1fcddc0a",
        strip_prefix = "bazel_gomock-fde78c91cf1783cc1e33ba278922ba67a6ee2a84",
    )
    http_archive(
        name = "kubernetes_helm",
        urls = [
            "https://get.helm.sh/helm-v2.17.0-linux-amd64.tar.gz",
        ],
        sha256 = "f3bec3c7c55f6a9eb9e6586b8c503f370af92fe987fcbf741f37707606d70296",
        strip_prefix = "linux-amd64",
        build_file = "//third_party/helm2:BUILD.bazel",
    )
    http_archive(
        name = "kubernetes_helm3",
        urls = [
            "https://get.helm.sh/helm-v3.9.0-linux-amd64.tar.gz",
        ],
        sha256 = "1484ffb0c7a608d8069470f48b88d729e88c41a1b6602f145231e8ea7b43b50a",
        strip_prefix = "linux-amd64",
        build_file = "//third_party/helm3:BUILD.bazel",
    )
    http_archive(
        name = "hashicorp_terraform",
        urls = [
            "https://releases.hashicorp.com/terraform/1.11.4/terraform_1.11.4_linux_amd64.zip",
        ],
        sha256 = "1ce994251c00281d6845f0f268637ba50c0005657eb3cf096b92f753b42ef4dc",
        build_file = "//third_party:terraform.BUILD",
    )
    http_archive(
        name = "com_github_kubernetes_sigs_application",
        urls = [
            "https://github.com/kubernetes-sigs/application/archive/c8e2959e57a02b3877b394984a288f9178977d8b.tar.gz",
        ],
        sha256 = "8bafd7fb97563d1a15d9afc68c87e3aabd664f60bd8005f1ae685d79842c1ac4",
        strip_prefix = "application-c8e2959e57a02b3877b394984a288f9178977d8b",
        build_file = "//third_party:app_crd.BUILD",
    )
    http_archive(
        name = "ingress-nginx",
        urls = [
            "https://github.com/kubernetes/ingress-nginx/archive/refs/tags/controller-v1.8.0.tar.gz",
        ],
        sha256 = "6e571764828b24545eea49582fd56d66d51fc66e52a375d98251c80c57fdb2fc",
        strip_prefix = "ingress-nginx-controller-v1.8.0",
        build_file = "//third_party:ingress-nginx.BUILD",
    )

# -- repo definitions -- #

non_module_deps = module_extension(implementation = _non_module_deps_impl)
