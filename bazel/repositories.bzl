load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def cloud_robotics_repositories():
    # Eigen
    # Based on https://github.com/tensorflow/tensorflow/blob/master/third_party/eigen.BUILD
    _maybe(
        http_archive,
        name = "org_tuxfamily_eigen",
        build_file = "@cloud_robotics//third_party:eigen.BUILD",
        sha256 = "ca7beac153d4059c02c8fc59816c82d54ea47fe58365e8aded4082ded0b820c4",
        strip_prefix = "eigen-eigen-f3a22f35b044",
        urls = [
            "http://mirror.bazel.build/bitbucket.org/eigen/eigen/get/f3a22f35b044.tar.gz",
            "https://bitbucket.org/eigen/eigen/get/f3a22f35b044.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "bazel_skylib",
        sha256 = "1c531376ac7e5a180e0237938a2536de0c54d93f5c278634818e0efc952dd56c",
        urls = [
            "https://mirror.bazel.build/github.com/bazelbuild/bazel-skylib/releases/download/1.0.3/bazel-skylib-1.0.3.tar.gz",
            "https://github.com/bazelbuild/bazel-skylib/releases/download/1.0.3/bazel-skylib-1.0.3.tar.gz",
        ],
    )

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

    # protos for the Google APIs
    _maybe(
        http_archive,
        name = "com_github_googleapis_googleapis",
        patch_args = ["-p1"],
        sha256 = "77216f166548374668a0a8ab1502acf3da6502affa826efca58cf6343b5550ed",
        strip_prefix = "googleapis-58ecdb1f0c0297975e68df5b45200a2d06f6d933",
        urls = [
            "https://mirror.bazel.build/github.com/googleapis/googleapis/archive/58ecdb1f0c0297975e68df5b45200a2d06f6d933.tar.gz",
            "https://github.com/googleapis/googleapis/archive/58ecdb1f0c0297975e68df5b45200a2d06f6d933.tar.gz",
        ],
    )

    # TODO(rodrigoq): rename to com_github_antonovvk_bazel_rules to match cartographer.
    _maybe(
        http_archive,
        name = "bazel_rules",
        sha256 = "2f5327a2dc9a0cc8ead93953a5d2ae2e0308aece685e46cc89c27538a7e9a73a",
        strip_prefix = "bazel_rules-c76e47ebe6f0a03b9dd99e245d5a0611813c36f9",
        urls = [
            "https://github.com/drigz/bazel_rules/archive/c76e47ebe6f0a03b9dd99e245d5a0611813c36f9.tar.gz",
        ],
    )

    # EmPy
    _maybe(
        http_archive,
        name = "empy_repo",
        build_file = "@cloud_robotics//third_party:empy.BUILD",
        sha256 = "9841e36dd26c7f69fe1005f9d9e078e41bdd50dd56fc77837ae390fb6af1aed7",
        strip_prefix = "empy-3.3.3",
        urls = [
            "https://mirror.bazel.build/www.alcyone.com/software/empy/empy-3.3.3.tar.gz",
            "http://www.alcyone.com/software/empy/empy-3.3.3.tar.gz",
        ],
    )

    # tinyxml 2.6.2 (mirrored on GitHub to avoid depending on SourceForge too)
    _maybe(
        http_archive,
        name = "com_github_icebreaker_tinyxml",
        build_file = "@cloud_robotics//third_party:tinyxml.BUILD",
        sha256 = "ea7db1f158f08dd0e4927112caaa620a341fea2e2717c99c5a807eca874f017d",
        strip_prefix = "TinyXML-b3e5aabf9c0272fdfd05487b5ea12eaf4522713d",
        urls = [
            "https://mirror.bazel.build/github.com/icebreaker/TinyXML/archive/b3e5aabf9c0272fdfd05487b5ea12eaf4522713d.tar.gz",
            "https://github.com/icebreaker/TinyXML/archive/b3e5aabf9c0272fdfd05487b5ea12eaf4522713d.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_leethomason_tinyxml2",
        build_file = "@cloud_robotics//third_party:tinyxml2.BUILD",
        sha256 = "a381729e32b6c2916a23544c04f342682d38b3f6e6c0cad3c25e900c3a7ef1a6",
        strip_prefix = "tinyxml2-7.0.1",
        urls = [
            "https://github.com/leethomason/tinyxml2/archive/7.0.1.tar.gz",
        ],
    )

    # zlib
    _maybe(
        http_archive,
        name = "net_zlib_zlib",
        build_file = "@cloud_robotics//third_party:zlib.BUILD",
        sha256 = "6d4d6640ca3121620995ee255945161821218752b551a1a180f4215f7d124d45",
        strip_prefix = "zlib-cacf7f1d4e3d44d871b605da3b647f07d718623f",
        urls = [
            "https://mirror.bazel.build/github.com/madler/zlib/archive/cacf7f1d4e3d44d871b605da3b647f07d718623f.tar.gz",
            "https://github.com/madler/zlib/archive/cacf7f1d4e3d44d871b605da3b647f07d718623f.tar.gz",
        ],
    )

    # bz2
    _maybe(
        http_archive,
        name = "org_bzip_bzip2",
        build_file = "@cloud_robotics//third_party:bzip2.BUILD",
        sha256 = "a2848f34fcd5d6cf47def00461fcb528a0484d8edef8208d6d2e2909dc61d9cd",
        strip_prefix = "bzip2-1.0.6",
        urls = [
            "https://mirror.bazel.build/www.bzip.org/1.0.6/bzip2-1.0.6.tar.gz",
            "http://www.bzip.org/1.0.6/bzip2-1.0.6.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_cyan4973_xxhash",
        build_file = "@cloud_robotics//third_party:xxhash.BUILD",
        sha256 = "d8c739ec666ac2af983a61dc932aaa2a8873df974d333a9922d472a121f2106e",
        strip_prefix = "xxHash-0.6.3",
        urls = ["https://github.com/Cyan4973/xxHash/archive/v0.6.3.tar.gz"],
    )

    _maybe(
        http_archive,
        name = "com_github_lz4_lz4",
        build_file = "@cloud_robotics//third_party:lz4.BUILD",
        sha256 = "2ca482ea7a9bb103603108b5a7510b7592b90158c151ff50a28f1ca8389fccf6",
        strip_prefix = "lz4-1.8.0",
        urls = ["https://github.com/lz4/lz4/archive/v1.8.0.tar.gz"],
    )

    # POCO
    _maybe(
        http_archive,
        name = "org_pocoproject_poco",
        build_file = "@cloud_robotics//third_party:poco.BUILD",
        sha256 = "40743cf18fadea6992e0ad7f668a75d46f08364a7f3ff748420fa080bbaaa3d1",
        strip_prefix = "poco-1.7.8p3",
        urls = [
            "https://mirror.bazel.build/pocoproject.org/releases/poco-1.7.8/poco-1.7.8p3.tar.gz",
            "https://pocoproject.org/releases/poco-1.7.8/poco-1.7.8p3.tar.gz",
        ],
    )

    # Boost
    SOURCEFORGE_MIRRORS = ["phoenixnap", "newcontinuum", "cfhcable", "superb-sea2", "cytranet", "iweb", "gigenet", "ayera", "astuteinternet", "pilotfiber", "svwh"]
    _maybe(
        http_archive,
        name = "boost",
        urls = [
            "https://%s.dl.sourceforge.net/project/boost/boost/1.63.0/boost_1_63_0.tar.gz" % m
            for m in SOURCEFORGE_MIRRORS
        ],
        build_file = "@com_github_nelhage_rules_boost//:BUILD.boost",
        strip_prefix = "boost_1_63_0",
        sha256 = "fe34a4e119798e10b8cc9e565b3b0284e9fd3977ec8a1b19586ad1dec397088b",
    )

    _maybe(
        http_archive,
        name = "com_github_nelhage_rules_boost",
        sha256 = "89ea32a8adb7521d06447c87bfb4517ff60f3834bd462c17c4b8f79047460410",
        strip_prefix = "rules_boost-f0ecfe836da225d4a396ef5f604e885ce70636fb",
        urls = [
            "https://github.com/nelhage/rules_boost/archive/f0ecfe836da225d4a396ef5f604e885ce70636fb.tar.gz",
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

    # Rules for building and handling Docker images with Bazel and define base images
    _maybe(
        http_archive,
        name = "io_bazel_rules_docker",
        sha256 = "85ffff62a4c22a74dbd98d05da6cf40f497344b3dbf1e1ab0a37ab2a1a6ca014",
        strip_prefix = "rules_docker-0.23.0",
        urls = ["https://github.com/bazelbuild/rules_docker/releases/download/v0.23.0/rules_docker-v0.23.0.tar.gz"],
    )

    # Go rules and proto support
    _maybe(
        http_archive,
        name = "io_bazel_rules_go",
        sha256 = "099a9fb96a376ccbbb7d291ed4ecbdfd42f6bc822ab77ae6f1b5cb9e914e94fa",
        urls = [
            "https://mirror.bazel.build/github.com/bazelbuild/rules_go/releases/download/v0.35.0/rules_go-v0.35.0.zip",
            "https://github.com/bazelbuild/rules_go/releases/download/v0.35.0/rules_go-v0.35.0.zip",
        ],
    )
    _maybe(
        http_archive,
        name = "bazel_gazelle",
        sha256 = "de69a09dc70417580aabf20a28619bb3ef60d038470c7cf8442fafcf627c21cb",
        urls = [
            "https://mirror.bazel.build/github.com/bazelbuild/bazel-gazelle/releases/download/v0.24.0/bazel-gazelle-v0.24.0.tar.gz",
            "https://github.com/bazelbuild/bazel-gazelle/releases/download/v0.24.0/bazel-gazelle-v0.24.0.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_bazelbuild_buildtools",
        sha256 = "e3bb0dc8b0274ea1aca75f1f8c0c835adbe589708ea89bf698069d0790701ea3",
        strip_prefix = "buildtools-5.1.0",
        urls = ["https://github.com/bazelbuild/buildtools/archive/5.1.0.tar.gz"],
    )

def _maybe(repo_rule, name, **kwargs):
    if name not in native.existing_rules():
        repo_rule(name = name, **kwargs)
