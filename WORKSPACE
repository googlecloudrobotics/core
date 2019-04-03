workspace(name = "cloud_robotics")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")
load("//bazel:repositories.bzl", "cloud_robotics_repositories")

# gazelle:repo bazel_gazelle
cloud_robotics_repositories()

# Protobuf
http_archive(
    name = "com_google_protobuf",
    sha256 = "f1748989842b46fa208b2a6e4e2785133cfcc3e4d43c17fecb023733f0f5443f",
    strip_prefix = "protobuf-3.7.1",
    urls = [
        "https://mirror.bazel.build/github.com/google/protobuf/archive/v3.7.1.tar.gz",
        "https://github.com/google/protobuf/archive/v3.7.1.tar.gz",
    ],
)

# These binds are required by com_google_protobuf.
bind(
    name = "zlib",
    actual = "@net_zlib_zlib//:zlib",
)

bind(
    name = "error_prone_annotations",
    actual = "//external:jar/com/google/errorprone/error_prone_annotations",
)

# gRPC Java binding
# If you update this, check if there are new versions of io.grpc:grpc-netty and
# io.grpc:grpc-services on Maven, and update the versions in
# maven_dependencies.yaml.
http_archive(
    name = "io_grpc_grpc_java",
    sha256 = "83f6fff2bb94e344a966cc687e672cfadf7b026b9c936d7034b6367d06bedc5c",
    strip_prefix = "grpc-java-1.17.2",
    urls = [
        "https://github.com/grpc/grpc-java/archive/v1.17.2.tar.gz",
    ],
)

load("@io_grpc_grpc_java//:repositories.bzl", "grpc_java_repositories")

# Maven is pulling in gRPC services and those dependencies mentioned below.
grpc_java_repositories(
    omit_com_google_api_grpc_google_common_protos = True,
    omit_com_google_auth_google_auth_library_credentials = True,
    omit_com_google_auth_google_auth_library_oauth2_http = True,
    omit_com_google_code_findbugs_jsr305 = True,
    omit_com_google_code_gson = True,
    omit_com_google_errorprone_error_prone_annotations = True,
    omit_com_google_guava = True,
    omit_com_google_j2objc_j2objc_annotations = True,
    omit_com_google_protobuf = True,
    omit_com_google_protobuf_javalite = True,
    omit_com_google_protobuf_nano_protobuf_javanano = True,
    omit_com_google_truth_truth = True,
    omit_com_squareup_okhttp = True,
    omit_com_squareup_okio = True,
    omit_io_netty_buffer = True,
    omit_io_netty_codec = True,
    omit_io_netty_codec_http = True,
    omit_io_netty_codec_http2 = True,
    omit_io_netty_codec_socks = True,
    omit_io_netty_common = True,
    omit_io_netty_handler = True,
    omit_io_netty_handler_proxy = True,
    omit_io_netty_resolver = True,
    omit_io_netty_tcnative_boringssl_static = True,
    omit_io_netty_transport = True,
    omit_io_opencensus_api = True,
    omit_io_opencensus_grpc_metrics = True,
    omit_javax_annotation = True,
    omit_junit_junit = True,
    omit_org_codehaus_mojo_animal_sniffer_annotations = True,
)

# Ensure that the protobuf_javalite version is new enough for recent Bazel
# versions.
# TODO(rodrigoq): remove this and omit_com_google_protobuf_javalite above,
# after https://github.com/grpc/grpc-java/pull/5212 is merged.
http_archive(
    name = "com_google_protobuf_javalite",
    sha256 = "97b07327b491924fc5173fe1adc2bb504751b0f13990b70b1b5da16eddb47c8d",
    strip_prefix = "protobuf-384989534b2246d413dbcd750744faab2607b516",
    urls = [
        "https://github.com/google/protobuf/archive/384989534b2246d413dbcd750744faab2607b516.tar.gz",
    ],
)

load(
    "@io_bazel_rules_docker//repositories:repositories.bzl",
    container_repositories = "repositories",
)

container_repositories()

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

container_pull(
    name = "turtlebot3-gazebo-cloud",
    digest = "sha256:a25d7bb358b6c0c86f4f3001e6cff6f51b4a30ac2d14d539e8dd1737f2c303f8",
    registry = "eu.gcr.io",
    repository = "cloud-robotics-releases",
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

load(
    "@io_bazel_rules_docker//python:image.bzl",
    _py_image_repos = "repositories",
)

_py_image_repos()

# Add Maven dependencies
load("//third_party:maven_dependencies.bzl", "maven_dependencies")

maven_dependencies()

# .deb for libpython2.7.so (required by rospack)
http_file(
    name = "libpython_2_7",
    downloaded_file_path = "libpython2.7_2.7.13-2+deb9u2_amd64.deb",
    sha256 = "032647c0cff6788b0d813f45e848a467f02090f358c7e056767cec041eedc3fa",
    urls = [
        "http://ftp.de.debian.org/debian/pool/main/p/python2.7/libpython2.7_2.7.13-2+deb9u2_amd64.deb",
        "http://snapshot.debian.org/archive/debian/20171129T213627Z/pool/main/p/python2.7/libpython2.7_2.7.13-2+deb9u2_amd64.deb",
    ],
)

# rules_python
# TODO(rodrigoq): revert back to upstream once this issue is resolved:
# https://github.com/bazelbuild/rules_python/issues/14
http_archive(
    name = "io_bazel_rules_python",
    sha256 = "2cd567d61cae56631db1e7ef39471f7904bbba1df4ba1f88da5bc014ca25e991",
    strip_prefix = "rules_python-0b13b922d7230c3eceb069140fae6beb5d1a44b9",
    urls = [
        "https://github.com/drigz/rules_python/archive/0b13b922d7230c3eceb069140fae6beb5d1a44b9.tar.gz",
    ],
)

load("@io_bazel_rules_python//python:pip.bzl", "pip_import", "pip_repositories")

pip_repositories()

pip_import(
    name = "ros_deps",
    requirements = "//third_party/ros:requirements.txt",
)

load("@ros_deps//:requirements.bzl", "pip_install")

pip_install()

pip_import(
    name = "ros_adapter_deps",
    requirements = "//src/python/ros_adapter:requirements.txt",
)

load("@ros_adapter_deps//:requirements.bzl", "pip_install")

pip_install()

pip_import(
    name = "ros_log_deps",
    requirements = "//src/python/ros_log:requirements.txt",
)

load("@ros_log_deps//:requirements.bzl", "pip_install")

pip_install()

pip_import(
    name = "ros_demo_deps",
    requirements = "//src/python/ros_demo:requirements.txt",
)

load("@ros_demo_deps//:requirements.bzl", "pip_install")

pip_install()

load("@io_bazel_rules_go//go:deps.bzl", "go_register_toolchains")
load("@bazel_gazelle//:deps.bzl", "go_repository")

# Do not call go_rules_dependencies() until after all other go_repository
# calls, or else the top-level definitions may be silently ignored.
# https://github.com/bazelbuild/bazel/issues/3908
go_register_toolchains()

load("@io_bazel_rules_go//go:deps.bzl", "go_rules_dependencies")

go_rules_dependencies()

load("@bazel_gazelle//:deps.bzl", "gazelle_dependencies")

gazelle_dependencies()

# Containerization rules for Go must come after go_rules_dependencies().
load(
    "@io_bazel_rules_docker//go:image.bzl",
    _go_image_repos = "repositories",
)

_go_image_repos()

load(
    "//bazel/proto_crd:repositories.bzl",
    proto_crd_repositories = "repositories",
)

proto_crd_repositories()

http_archive(
    name = "com_github_kubernetes_sigs_application",
    build_file = "@cloud_robotics//third_party:app_crd.BUILD",
    sha256 = "50cd3e3427355f77d177f7cee7704b1f077a2564e67d07db762bdb3022c6cb58",
    strip_prefix = "application-464aad9c212082e99a9f9ed4515f1bd1f1df2bfa",
    urls = [
        "https://github.com/kubernetes-sigs/application/archive/464aad9c212082e99a9f9ed4515f1bd1f1df2bfa.tar.gz",
    ],
)

# Third party Go libraries

# These are imported manually as they are not used directly in the Go tree but
# by bazel rules.
go_repository(
    name = "com_github_ryanuber_go_license",
    commit = "3878fc8f33ea4637cf02cf8a0bf926288a246ceb",
    importpath = "github.com/ryanuber/go-license",
)

go_repository(
    name = "bazel_gomock",
    commit = "76ed30b11d0071841ba258f009178407b1a13d68",
    importpath = "github.com/jmhodges/bazel_gomock",
)

# Generated by gazelle.
go_repository(
    name = "co_honnef_go_tools",
    commit = "c2f93a96b099",
    importpath = "honnef.co/go/tools",
)

go_repository(
    name = "com_github_alecthomas_template",
    commit = "a0175ee3bccc",
    importpath = "github.com/alecthomas/template",
)

go_repository(
    name = "com_github_alecthomas_units",
    commit = "2efee857e7cf",
    importpath = "github.com/alecthomas/units",
)

go_repository(
    name = "com_github_anmitsu_go_shlex",
    commit = "648efa622239",
    importpath = "github.com/anmitsu/go-shlex",
)

go_repository(
    name = "com_github_apache_thrift",
    importpath = "github.com/apache/thrift",
    tag = "v0.12.0",
)

go_repository(
    name = "com_github_apcera_libretto",
    importpath = "github.com/apcera/libretto",
    tag = "v0.11.0",
)

go_repository(
    name = "com_github_appscode_jsonpatch",
    commit = "7c0e3b262f30",
    importpath = "github.com/appscode/jsonpatch",
)

go_repository(
    name = "com_github_beorn7_perks",
    commit = "3a771d992973",
    importpath = "github.com/beorn7/perks",
)

go_repository(
    name = "com_github_bradfitz_go_smtpd",
    commit = "deb6d6237625",
    importpath = "github.com/bradfitz/go-smtpd",
)

go_repository(
    name = "com_github_burntsushi_toml",
    importpath = "github.com/BurntSushi/toml",
    tag = "v0.3.1",
)

go_repository(
    name = "com_github_client9_misspell",
    importpath = "github.com/client9/misspell",
    tag = "v0.3.4",
)

go_repository(
    name = "com_github_coreos_go_systemd",
    commit = "c6f51f82210d",
    importpath = "github.com/coreos/go-systemd",
)

go_repository(
    name = "com_github_cyphar_filepath_securejoin",
    importpath = "github.com/cyphar/filepath-securejoin",
    tag = "v0.2.2",
)

go_repository(
    name = "com_github_davecgh_go_spew",
    importpath = "github.com/davecgh/go-spew",
    tag = "v1.1.1",
)

go_repository(
    name = "com_github_eapache_go_resiliency",
    importpath = "github.com/eapache/go-resiliency",
    tag = "v1.1.0",
)

go_repository(
    name = "com_github_eapache_go_xerial_snappy",
    commit = "776d5712da21",
    importpath = "github.com/eapache/go-xerial-snappy",
)

go_repository(
    name = "com_github_eapache_queue",
    importpath = "github.com/eapache/queue",
    tag = "v1.1.0",
)

go_repository(
    name = "com_github_emicklei_go_restful",
    importpath = "github.com/emicklei/go-restful",
    tag = "v2.9.3",
)

go_repository(
    name = "com_github_evanphx_json_patch",
    importpath = "github.com/evanphx/json-patch",
    tag = "v4.0.0",
)

go_repository(
    name = "com_github_flynn_go_shlex",
    commit = "3f9db97f8568",
    importpath = "github.com/flynn/go-shlex",
)

go_repository(
    name = "com_github_fsnotify_fsnotify",
    importpath = "github.com/fsnotify/fsnotify",
    tag = "v1.4.7",
)

go_repository(
    name = "com_github_ghodss_yaml",
    importpath = "github.com/ghodss/yaml",
    tag = "v1.0.0",
)

go_repository(
    name = "com_github_gliderlabs_ssh",
    importpath = "github.com/gliderlabs/ssh",
    tag = "v0.1.1",
)

go_repository(
    name = "com_github_go_kit_kit",
    importpath = "github.com/go-kit/kit",
    tag = "v0.8.0",
)

go_repository(
    name = "com_github_go_logfmt_logfmt",
    importpath = "github.com/go-logfmt/logfmt",
    tag = "v0.3.0",
)

go_repository(
    name = "com_github_go_logr_logr",
    importpath = "github.com/go-logr/logr",
    tag = "v0.1.0",
)

go_repository(
    name = "com_github_go_logr_zapr",
    importpath = "github.com/go-logr/zapr",
    tag = "v0.1.1",
)

go_repository(
    name = "com_github_go_openapi_jsonpointer",
    importpath = "github.com/go-openapi/jsonpointer",
    tag = "v0.17.0",
)

go_repository(
    name = "com_github_go_openapi_jsonreference",
    importpath = "github.com/go-openapi/jsonreference",
    tag = "v0.17.0",
)

go_repository(
    name = "com_github_go_openapi_spec",
    importpath = "github.com/go-openapi/spec",
    tag = "v0.19.0",
)

go_repository(
    name = "com_github_go_openapi_swag",
    importpath = "github.com/go-openapi/swag",
    tag = "v0.17.0",
)

go_repository(
    name = "com_github_go_stack_stack",
    importpath = "github.com/go-stack/stack",
    tag = "v1.8.0",
)

go_repository(
    name = "com_github_gobwas_glob",
    importpath = "github.com/gobwas/glob",
    tag = "v0.2.3",
)

go_repository(
    name = "com_github_gogo_protobuf",
    importpath = "github.com/gogo/protobuf",
    tag = "v1.2.1",
)

go_repository(
    name = "com_github_golang_glog",
    commit = "23def4e6c14b",
    importpath = "github.com/golang/glog",
)

go_repository(
    name = "com_github_golang_groupcache",
    commit = "5b532d6fd5ef",
    importpath = "github.com/golang/groupcache",
)

go_repository(
    name = "com_github_golang_lint",
    commit = "06c8688daad7",
    importpath = "github.com/golang/lint",
)

go_repository(
    name = "com_github_golang_mock",
    importpath = "github.com/golang/mock",
    tag = "v1.2.0",
)

go_repository(
    name = "com_github_golang_protobuf",
    importpath = "github.com/golang/protobuf",
    tag = "v1.3.1",
)

go_repository(
    name = "com_github_golang_snappy",
    commit = "2e65f85255db",
    importpath = "github.com/golang/snappy",
)

go_repository(
    name = "com_github_google_btree",
    importpath = "github.com/google/btree",
    tag = "v1.0.0",
)

go_repository(
    name = "com_github_google_go_cmp",
    importpath = "github.com/google/go-cmp",
    tag = "v0.2.0",
)

go_repository(
    name = "com_github_google_go_github",
    importpath = "github.com/google/go-github",
    tag = "v17.0.0",
)

go_repository(
    name = "com_github_google_go_querystring",
    importpath = "github.com/google/go-querystring",
    tag = "v1.0.0",
)

go_repository(
    name = "com_github_google_gofuzz",
    commit = "24818f796faf",
    importpath = "github.com/google/gofuzz",
)

go_repository(
    name = "com_github_google_martian",
    importpath = "github.com/google/martian",
    tag = "v2.1.0",
)

go_repository(
    name = "com_github_google_pprof",
    commit = "3ea8567a2e57",
    importpath = "github.com/google/pprof",
)

go_repository(
    name = "com_github_google_uuid",
    importpath = "github.com/google/uuid",
    tag = "v1.1.1",
)

go_repository(
    name = "com_github_googleapis_gax_go",
    importpath = "github.com/googleapis/gax-go",
    tag = "v2.0.2",
)

go_repository(
    name = "com_github_googleapis_gax_go_v2",
    importpath = "github.com/googleapis/gax-go/v2",
    tag = "v2.0.4",
)

go_repository(
    name = "com_github_googleapis_gnostic",
    importpath = "github.com/googleapis/gnostic",
    tag = "v0.2.0",
)

go_repository(
    name = "com_github_gorilla_context",
    importpath = "github.com/gorilla/context",
    tag = "v1.1.1",
)

go_repository(
    name = "com_github_gorilla_mux",
    importpath = "github.com/gorilla/mux",
    tag = "v1.6.2",
)

go_repository(
    name = "com_github_gregjones_httpcache",
    commit = "3befbb6ad0cc",
    importpath = "github.com/gregjones/httpcache",
)

go_repository(
    name = "com_github_h2non_parth",
    commit = "b4df798d6542",
    importpath = "github.com/h2non/parth",
)

go_repository(
    name = "com_github_hashicorp_golang_lru",
    importpath = "github.com/hashicorp/golang-lru",
    tag = "v0.5.0",
)

go_repository(
    name = "com_github_hpcloud_tail",
    importpath = "github.com/hpcloud/tail",
    tag = "v1.0.0",
)

go_repository(
    name = "com_github_huandu_xstrings",
    importpath = "github.com/huandu/xstrings",
    tag = "v1.2.0",
)

go_repository(
    name = "com_github_imdario_mergo",
    importpath = "github.com/imdario/mergo",
    tag = "v0.3.7",
)

go_repository(
    name = "com_github_inconshreveable_mousetrap",
    importpath = "github.com/inconshreveable/mousetrap",
    tag = "v1.0.0",
)

go_repository(
    name = "com_github_jellevandenhooff_dkim",
    commit = "f50fe3d243e1",
    importpath = "github.com/jellevandenhooff/dkim",
)

go_repository(
    name = "com_github_jhump_protoreflect",
    importpath = "github.com/jhump/protoreflect",
    tag = "v1.2.0",
)

go_repository(
    name = "com_github_json_iterator_go",
    importpath = "github.com/json-iterator/go",
    tag = "v1.1.6",
)

go_repository(
    name = "com_github_jstemmer_go_junit_report",
    commit = "af01ea7f8024",
    importpath = "github.com/jstemmer/go-junit-report",
)

go_repository(
    name = "com_github_jteeuwen_go_bindata",
    commit = "6025e8de665b",
    importpath = "github.com/jteeuwen/go-bindata",
)

go_repository(
    name = "com_github_julienschmidt_httprouter",
    importpath = "github.com/julienschmidt/httprouter",
    tag = "v1.2.0",
)

go_repository(
    name = "com_github_kisielk_errcheck",
    importpath = "github.com/kisielk/errcheck",
    tag = "v1.1.0",
)

go_repository(
    name = "com_github_kisielk_gotool",
    importpath = "github.com/kisielk/gotool",
    tag = "v1.0.0",
)

go_repository(
    name = "com_github_konsorten_go_windows_terminal_sequences",
    importpath = "github.com/konsorten/go-windows-terminal-sequences",
    tag = "v1.0.1",
)

go_repository(
    name = "com_github_kr_logfmt",
    commit = "b84e30acd515",
    importpath = "github.com/kr/logfmt",
)

go_repository(
    name = "com_github_kr_pretty",
    importpath = "github.com/kr/pretty",
    tag = "v0.1.0",
)

go_repository(
    name = "com_github_kr_pty",
    importpath = "github.com/kr/pty",
    tag = "v1.1.1",
)

go_repository(
    name = "com_github_kr_text",
    importpath = "github.com/kr/text",
    tag = "v0.1.0",
)

go_repository(
    name = "com_github_mailru_easyjson",
    commit = "60711f1a8329",
    importpath = "github.com/mailru/easyjson",
)

go_repository(
    name = "com_github_masterminds_goutils",
    importpath = "github.com/Masterminds/goutils",
    tag = "v1.1.0",
)

go_repository(
    name = "com_github_masterminds_semver",
    importpath = "github.com/Masterminds/semver",
    tag = "v1.4.2",
)

go_repository(
    name = "com_github_masterminds_sprig",
    importpath = "github.com/Masterminds/sprig",
    tag = "v2.18.0",
)

go_repository(
    name = "com_github_matttproud_golang_protobuf_extensions",
    importpath = "github.com/matttproud/golang_protobuf_extensions",
    tag = "v1.0.1",
)

go_repository(
    name = "com_github_modern_go_concurrent",
    commit = "bacd9c7ef1dd",
    importpath = "github.com/modern-go/concurrent",
)

go_repository(
    name = "com_github_modern_go_reflect2",
    importpath = "github.com/modern-go/reflect2",
    tag = "v1.0.1",
)

go_repository(
    name = "com_github_motemen_go_loghttp",
    commit = "974ac5ceac27",
    importpath = "github.com/motemen/go-loghttp",
)

go_repository(
    name = "com_github_motemen_go_nuts",
    commit = "42c35bdb11c2",
    importpath = "github.com/motemen/go-nuts",
)

go_repository(
    name = "com_github_mwitkow_go_conntrack",
    commit = "cc309e4a2223",
    importpath = "github.com/mwitkow/go-conntrack",
)

go_repository(
    name = "com_github_nbio_st",
    commit = "e9e8d9816f32",
    importpath = "github.com/nbio/st",
)

go_repository(
    name = "com_github_onsi_ginkgo",
    importpath = "github.com/onsi/ginkgo",
    tag = "v1.7.0",
)

go_repository(
    name = "com_github_onsi_gomega",
    importpath = "github.com/onsi/gomega",
    tag = "v1.5.0",
)

go_repository(
    name = "com_github_openzipkin_zipkin_go",
    importpath = "github.com/openzipkin/zipkin-go",
    tag = "v0.1.6",
)

go_repository(
    name = "com_github_pierrec_lz4",
    importpath = "github.com/pierrec/lz4",
    tag = "v2.0.5",
)

go_repository(
    name = "com_github_pkg_errors",
    importpath = "github.com/pkg/errors",
    tag = "v0.8.1",
)

go_repository(
    name = "com_github_pmezard_go_difflib",
    importpath = "github.com/pmezard/go-difflib",
    tag = "v1.0.0",
)

go_repository(
    name = "com_github_prometheus_client_golang",
    commit = "3c4408c8b829",
    importpath = "github.com/prometheus/client_golang",
)

go_repository(
    name = "com_github_prometheus_client_model",
    commit = "56726106282f",
    importpath = "github.com/prometheus/client_model",
)

go_repository(
    name = "com_github_prometheus_common",
    importpath = "github.com/prometheus/common",
    tag = "v0.2.0",
)

go_repository(
    name = "com_github_prometheus_procfs",
    commit = "bf6a532e95b1",
    importpath = "github.com/prometheus/procfs",
)

go_repository(
    name = "com_github_puerkitobio_purell",
    importpath = "github.com/PuerkitoBio/purell",
    tag = "v1.1.0",
)

go_repository(
    name = "com_github_puerkitobio_urlesc",
    commit = "de5bf2ad4578",
    importpath = "github.com/PuerkitoBio/urlesc",
)

go_repository(
    name = "com_github_rcrowley_go_metrics",
    commit = "3113b8401b8a",
    importpath = "github.com/rcrowley/go-metrics",
)

go_repository(
    name = "com_github_shopify_sarama",
    importpath = "github.com/Shopify/sarama",
    tag = "v1.19.0",
)

go_repository(
    name = "com_github_shopify_toxiproxy",
    importpath = "github.com/Shopify/toxiproxy",
    tag = "v2.1.4",
)

go_repository(
    name = "com_github_sirupsen_logrus",
    importpath = "github.com/sirupsen/logrus",
    tag = "v1.2.0",
)

go_repository(
    name = "com_github_spf13_cobra",
    importpath = "github.com/spf13/cobra",
    tag = "v0.0.3",
)

go_repository(
    name = "com_github_spf13_pflag",
    importpath = "github.com/spf13/pflag",
    tag = "v1.0.3",
)

go_repository(
    name = "com_github_stretchr_objx",
    importpath = "github.com/stretchr/objx",
    tag = "v0.1.1",
)

go_repository(
    name = "com_github_stretchr_testify",
    importpath = "github.com/stretchr/testify",
    tag = "v1.2.2",
)

go_repository(
    name = "com_github_tarm_serial",
    commit = "98f6abe2eb07",
    importpath = "github.com/tarm/serial",
)

go_repository(
    name = "com_google_cloud_go",
    importpath = "cloud.google.com/go",
    tag = "v0.34.0",
)

go_repository(
    name = "in_gopkg_airbrake_gobrake_v2",
    importpath = "gopkg.in/airbrake/gobrake.v2",
    tag = "v2.0.9",
)

go_repository(
    name = "in_gopkg_alecthomas_kingpin_v2",
    importpath = "gopkg.in/alecthomas/kingpin.v2",
    tag = "v2.2.6",
)

go_repository(
    name = "in_gopkg_check_v1",
    commit = "788fd7840127",
    importpath = "gopkg.in/check.v1",
)

go_repository(
    name = "in_gopkg_fsnotify_v1",
    importpath = "gopkg.in/fsnotify.v1",
    tag = "v1.4.7",
)

go_repository(
    name = "in_gopkg_gemnasium_logrus_airbrake_hook_v2",
    importpath = "gopkg.in/gemnasium/logrus-airbrake-hook.v2",
    tag = "v2.1.2",
)

go_repository(
    name = "in_gopkg_h2non_gock_v1",
    importpath = "gopkg.in/h2non/gock.v1",
    tag = "v1.0.14",
)

go_repository(
    name = "in_gopkg_inf_v0",
    importpath = "gopkg.in/inf.v0",
    tag = "v0.9.1",
)

go_repository(
    name = "in_gopkg_tomb_v1",
    commit = "dd632973f1e7",
    importpath = "gopkg.in/tomb.v1",
)

go_repository(
    name = "in_gopkg_yaml_v2",
    importpath = "gopkg.in/yaml.v2",
    tag = "v2.2.2",
)

go_repository(
    name = "io_k8s_api",
    build_file_proto_mode = "disable",
    commit = "67edc246be36",
    importpath = "k8s.io/api",
)

go_repository(
    name = "io_k8s_apiextensions_apiserver",
    build_extra_args = ["-exclude=vendor"],
    build_file_proto_mode = "disable",
    commit = "4cac3cbacb4e",
    importpath = "k8s.io/apiextensions-apiserver",
)

go_repository(
    name = "io_k8s_apimachinery",
    build_extra_args = ["-exclude=vendor"],
    build_file_proto_mode = "disable",
    commit = "2b1284ed4c93",
    importpath = "k8s.io/apimachinery",
)

go_repository(
    name = "io_k8s_client_go",
    build_extra_args = ["-exclude=vendor"],
    build_file_proto_mode = "disable",
    importpath = "k8s.io/client-go",
    tag = "v10.0.0",
)

go_repository(
    name = "io_k8s_code_generator",
    commit = "405721ab9678",
    importpath = "k8s.io/code-generator",
)

go_repository(
    name = "io_k8s_gengo",
    commit = "fd15ee9cc2f7",
    importpath = "k8s.io/gengo",
)

go_repository(
    name = "io_k8s_helm",
    importpath = "k8s.io/helm",
    tag = "v2.13.1",
)

go_repository(
    name = "io_k8s_klog",
    importpath = "k8s.io/klog",
    tag = "v0.2.0",
)

go_repository(
    name = "io_k8s_kube_openapi",
    commit = "94e1e7b7574c",
    importpath = "k8s.io/kube-openapi",
)

go_repository(
    name = "io_k8s_sigs_controller_runtime",
    build_extra_args = ["-exclude=vendor"],
    importpath = "sigs.k8s.io/controller-runtime",
    tag = "v0.1.10",
)

go_repository(
    name = "io_k8s_sigs_kind",
    commit = "2d52d5cc5e2b",
    importpath = "sigs.k8s.io/kind",
)

go_repository(
    name = "io_k8s_sigs_kustomize",
    importpath = "sigs.k8s.io/kustomize",
    tag = "v2.0.3",
)

go_repository(
    name = "io_k8s_sigs_testing_frameworks",
    importpath = "sigs.k8s.io/testing_frameworks",
    tag = "v0.1.1",
)

go_repository(
    name = "io_k8s_sigs_yaml",
    importpath = "sigs.k8s.io/yaml",
    tag = "v1.1.0",
)

go_repository(
    name = "io_k8s_utils",
    commit = "21c4ce38f2a7",
    importpath = "k8s.io/utils",
)

go_repository(
    name = "io_opencensus_go",
    importpath = "go.opencensus.io",
    tag = "v0.20.0",
)

go_repository(
    name = "org_go4",
    commit = "417644f6feb5",
    importpath = "go4.org",
)

go_repository(
    name = "org_go4_grpc",
    commit = "11d0a25b4919",
    importpath = "grpc.go4.org",
)

go_repository(
    name = "org_golang_google_api",
    importpath = "google.golang.org/api",
    tag = "v0.3.0",
)

go_repository(
    name = "org_golang_google_appengine",
    importpath = "google.golang.org/appengine",
    tag = "v1.4.0",
)

go_repository(
    name = "org_golang_google_genproto",
    commit = "f467c93bbac2",
    importpath = "google.golang.org/genproto",
)

go_repository(
    name = "org_golang_google_grpc",
    importpath = "google.golang.org/grpc",
    tag = "v1.19.1",
)

go_repository(
    name = "org_golang_x_build",
    commit = "5284462c4bec",
    importpath = "golang.org/x/build",
)

go_repository(
    name = "org_golang_x_crypto",
    commit = "a5d413f7728c",
    importpath = "golang.org/x/crypto",
)

go_repository(
    name = "org_golang_x_exp",
    commit = "509febef88a4",
    importpath = "golang.org/x/exp",
)

go_repository(
    name = "org_golang_x_lint",
    commit = "5614ed5bae6f",
    importpath = "golang.org/x/lint",
)

go_repository(
    name = "org_golang_x_net",
    commit = "74de082e2cca",
    importpath = "golang.org/x/net",
)

go_repository(
    name = "org_golang_x_oauth2",
    commit = "9f3314589c9a",
    importpath = "golang.org/x/oauth2",
)

go_repository(
    name = "org_golang_x_perf",
    commit = "6e6d33e29852",
    importpath = "golang.org/x/perf",
)

go_repository(
    name = "org_golang_x_sync",
    commit = "e225da77a7e6",
    importpath = "golang.org/x/sync",
)

go_repository(
    name = "org_golang_x_sys",
    commit = "d0b11bdaac8a",
    importpath = "golang.org/x/sys",
)

go_repository(
    name = "org_golang_x_text",
    importpath = "golang.org/x/text",
    tag = "v0.3.0",
)

go_repository(
    name = "org_golang_x_time",
    commit = "9d24e82272b4",
    importpath = "golang.org/x/time",
)

go_repository(
    name = "org_golang_x_tools",
    commit = "e65039ee4138",
    importpath = "golang.org/x/tools",
)

go_repository(
    name = "org_uber_go_atomic",
    importpath = "go.uber.org/atomic",
    tag = "v1.3.2",
)

go_repository(
    name = "org_uber_go_multierr",
    importpath = "go.uber.org/multierr",
    tag = "v1.1.0",
)

go_repository(
    name = "org_uber_go_zap",
    importpath = "go.uber.org/zap",
    tag = "v1.9.1",
)

go_repository(
    name = "com_github_pborman_uuid",
    importpath = "github.com/pborman/uuid",
    tag = "v1.2.0",
)

go_repository(
    name = "com_github_peterbourgon_diskv",
    importpath = "github.com/peterbourgon/diskv",
    tag = "v2.0.1",
)
