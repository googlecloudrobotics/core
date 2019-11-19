load("@bazel_gazelle//:deps.bzl", "gazelle_dependencies", "go_repository")
load("@com_github_bazelbuild_buildtools//buildifier:deps.bzl", "buildifier_dependencies")
load("@io_bazel_rules_go//go:deps.bzl", "go_register_toolchains")
load("@io_bazel_rules_go//go:deps.bzl", "go_rules_dependencies")

def repositories():
    # Bazel build tools (must be before go_rules_dependencies() to take priority)
    buildifier_dependencies()

    # Do not call go_rules_dependencies() until after all other go_repository
    # calls, or else the top-level definitions may be silently ignored.
    # https://github.com/bazelbuild/bazel/issues/3908
    go_register_toolchains()

    _maybe(
        go_repository,
        name = "com_github_go_openapi_jsonpointer",
        commit = "ef5f0afec364d3b9396b7b77b43dbe26bf1f8004",
        importpath = "github.com/go-openapi/jsonpointer",
    )

    _maybe(
        go_repository,
        name = "com_github_go_openapi_jsonreference",
        commit = "8483a886a90412cd6858df4ea3483dce9c8e35a3",
        importpath = "github.com/go-openapi/jsonreference",
    )

    _maybe(
        go_repository,
        name = "com_github_go_openapi_spec",
        commit = "5b6cdde3200976e3ecceb2868706ee39b6aff3e4",
        importpath = "github.com/go-openapi/spec",
    )

    _maybe(
        go_repository,
        name = "com_github_go_openapi_swag",
        commit = "1d29f06aebd59ccdf11ae04aa0334ded96e2d909",
        importpath = "github.com/go-openapi/swag",
    )

    _maybe(
        go_repository,
        name = "com_github_ghodss_yaml",
        commit = "0ca9ea5df5451ffdf184b4428c902747c2c11cd7",
        importpath = "github.com/ghodss/yaml",
    )

    _maybe(
        go_repository,
        name = "com_github_google_gofuzz",
        commit = "24818f796faf91cd76ec7bddd72458fbced7a6c1",
        importpath = "github.com/google/gofuzz",
    )

    _maybe(
        go_repository,
        name = "com_github_json_iterator_go",
        commit = "ab8a2e0c74be9d3be70b3184d9acc634935ded82",
        importpath = "github.com/json-iterator/go",
    )

    _maybe(
        go_repository,
        name = "com_github_mailru_easyjson",
        commit = "60711f1a8329503b04e1c88535f419d0bb440bff",
        importpath = "github.com/mailru/easyjson",
    )

    _maybe(
        go_repository,
        name = "com_github_modern_go_concurrent",
        commit = "bacd9c7ef1dd9b15be4a9909b8ac7a4e313eec94",
        importpath = "github.com/modern-go/concurrent",
    )

    _maybe(
        go_repository,
        name = "com_github_modern_go_reflect2",
        commit = "4b7aa43c6742a2c18fdef89dd197aaae7dac7ccd",
        importpath = "github.com/modern-go/reflect2",
    )

    _maybe(
        go_repository,
        name = "com_github_puerkitobio_purell",
        commit = "0bcb03f4b4d0a9428594752bd2a3b9aa0a9d4bd4",
        importpath = "github.com/PuerkitoBio/purell",
    )

    _maybe(
        go_repository,
        name = "com_github_puerkitobio_urlesc",
        commit = "de5bf2ad457846296e2031421a34e2568e304e35",
        importpath = "github.com/PuerkitoBio/urlesc",
    )

    _maybe(
        go_repository,
        name = "grpc_ecosystem_grpc_gateway",
        commit = "50c55a9810a974dc5a9e7dd1a5c0d295d525f283",
        importpath = "github.com/grpc-ecosystem/grpc-gateway",
    )

    _maybe(
        go_repository,
        name = "io_k8s_apiextensions_apiserver",
        # Ignore the vendored k8s deps, since we pull in compatible versions in WORKSPACE.
        build_extra_args = ["-exclude=vendor/k8s.io"],
        build_file_proto_mode = "disable",
        commit = "7d26de67f177df719a36756e658973478df68485",
        importpath = "k8s.io/apiextensions-apiserver",
    )

    _maybe(
        go_repository,
        name = "in_gopkg_inf_v0",
        commit = "d2d2541c53f18d2a059457998ce2876cc8e67cbf",
        importpath = "gopkg.in/inf.v0",
    )

    _maybe(
        go_repository,
        name = "in_gopkg_yaml_v2",
        commit = "5420a8b6744d3b0345ab293f6fcba19c978f1183",
        importpath = "gopkg.in/yaml.v2",
    )

    _maybe(
        go_repository,
        name = "io_k8s_apimachinery",
        build_file_proto_mode = "disable",
        commit = "2b1284ed4c93a43499e781493253e2ac5959c4fd",
        importpath = "k8s.io/apimachinery",
    )

    _maybe(
        go_repository,
        name = "io_k8s_klog",
        commit = "a5bc97fbc634d635061f3146511332c7e313a55a",
        importpath = "k8s.io/klog",
    )

    _maybe(
        go_repository,
        name = "io_k8s_sigs_yaml",
        commit = "fd68e9863619f6ec2fdd8625fe1f02e7c877e480",
        importpath = "sigs.k8s.io/yaml",
    )

    go_rules_dependencies()

    gazelle_dependencies()

def _maybe(repo_rule, name, **kwargs):
    if name not in native.existing_rules():
        repo_rule(name = name, **kwargs)
