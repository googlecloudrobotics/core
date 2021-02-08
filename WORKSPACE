workspace(name = "cloud_robotics")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")
load("//bazel:repositories.bzl", "cloud_robotics_repositories")

# gazelle:repo bazel_gazelle
cloud_robotics_repositories()

load("@com_google_protobuf//:protobuf_deps.bzl", "protobuf_deps")

protobuf_deps()

# gRPC Java binding
# If you update this, check if there are new versions of io.grpc:grpc-netty and
# io.grpc:grpc-services on Maven, and update the versions in
# maven_dependencies.yaml.
# Note: we pin a commit shortly after 1.26, because our CI expects `bazel fetch
# //...` to work.
http_archive(
    name = "io_grpc_grpc_java",
    patch_args = ["-p1"],
    patches = ["//third_party:io_grpc_grpc_java-d3c77f2d870baf8c8340890eb5aed590a5f3940c.patch"],
    sha256 = "982e892c339364c83fbf95f17a7d9898594c35a23d9dea894a2d014834eb5da4",
    strip_prefix = "grpc-java-274bf62e04f66fe9e0ffa4cac052a145e7c7b690",
    urls = [
        "https://github.com/grpc/grpc-java/archive/274bf62e04f66fe9e0ffa4cac052a145e7c7b690.tar.gz",
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

load(
    "@io_bazel_rules_docker//repositories:repositories.bzl",
    container_repositories = "repositories",
)

container_repositories()

load("@io_bazel_rules_docker//repositories:deps.bzl", container_deps = "deps")

container_deps()

load("@io_bazel_rules_docker//repositories:pip_repositories.bzl", "pip_deps")

pip_deps()

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

http_archive(
    name = "com_github_kubernetes_sigs_application",
    build_file = "@cloud_robotics//third_party:app_crd.BUILD",
    sha256 = "316b88383ef635a130b48c0519154274e72c7298b09c1425c736a2bbdd00c021",
    strip_prefix = "application-0.8.3",
    urls = [
        "https://github.com/kubernetes-sigs/application/archive/v0.8.3.tar.gz",
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
    name = "com_github_alessio_shellescape",
    commit = "52074bc9df61e0d2af85c160b3a325403955d7fb",
    importpath = "github.com/alessio/shellescape",
)

go_repository(
    name = "com_github_aokoli_goutils",
    commit = "41ac8693c5c10a92ea1ff5ac3a7f95646f6123b0",
    importpath = "github.com/aokoli/goutils",
)

go_repository(
    name = "com_github_apcera_libretto",
    commit = "3178799fbb1e39c74b02e3ecf46330b3ef0ed486",
    importpath = "github.com/apcera/libretto",
)

go_repository(
    name = "com_github_appscode_jsonpatch",
    commit = "7c0e3b262f30165a8ec3d0b4c6059fd92703bfb2",
    importpath = "github.com/appscode/jsonpatch",
)

go_repository(
    name = "com_github_azure_go_autorest",
    commit = "4b7f49dc5db2e1e6d528524d269b4181981a7ebf",
    importpath = "github.com/Azure/go-autorest",
)

go_repository(
    name = "com_github_beorn7_perks",
    commit = "3a771d992973f24aa725d07868b467d1ddfceafb",
    importpath = "github.com/beorn7/perks",
)

go_repository(
    name = "com_github_burntsushi_toml",
    commit = "3012a1dbe2e4bd1391d42b32f0577cb7bbc7f005",
    importpath = "github.com/BurntSushi/toml",
)

go_repository(
    name = "com_github_chai2010_gettext_go",
    commit = "bf70f2a70fb1b1f36d90d671a72795984eab0fcb",
    importpath = "github.com/chai2010/gettext-go",
)

go_repository(
    name = "com_github_cenkalti_backoff",
    commit = "2146c93394225c3732078705043ce9f26584d334",
    importpath = "github.com/cenkalti/backoff",
)

go_repository(
    name = "com_github_cyphar_filepath_securejoin",
    commit = "a261ee33d7a517f054effbf451841abaafe3e0fd",
    importpath = "github.com/cyphar/filepath-securejoin",
)

go_repository(
    name = "com_github_davecgh_go_spew",
    commit = "8991bc29aa16c548c550c7ff78260e27b9ab7c73",
    importpath = "github.com/davecgh/go-spew",
)

go_repository(
    name = "com_github_dgrijalva_jwt_go",
    commit = "06ea1031745cb8b3dab3f6a236daf2b0aa468b7e",
    importpath = "github.com/dgrijalva/jwt-go",
)

go_repository(
    name = "com_github_emicklei_go_restful",
    commit = "85d198d05a92d31823b852b4a5928114912e8949",
    importpath = "github.com/emicklei/go-restful",
)

go_repository(
    name = "com_github_evanphx_json_patch",
    commit = "162e5629780b286cc21cd80f1114a71f5917951f",
    importpath = "github.com/evanphx/json-patch",
)

go_repository(
    name = "com_github_exponent_io_jsonpath",
    commit = "d6023ce2651d8eafb5c75bb0c7167536102ec9f5",
    importpath = "github.com/exponent-io/jsonpath",
)

go_repository(
    name = "com_github_fsnotify_fsnotify",
    commit = "c2828203cd70a50dcccfb2761f8b1f8ceef9a8e9",
    importpath = "github.com/fsnotify/fsnotify",
)

go_repository(
    name = "in_gopkg_fsnotify_v1",
    commit = "45d7d09e39ef4ac08d493309fa031790c15bfe8a",
    importpath = "gopkg.in/fsnotify.v1",
    remote = "https://github.com/fsnotify/fsnotify",
    vcs = "git",
)

go_repository(
    name = "com_github_getlantern_httptest",
    commit = "4b40f4c7e590b2afa94729ca7f738e0b14b13749",
    importpath = "github.com/getlantern/httptest",
)

go_repository(
    name = "com_github_getlantern_mockconn",
    commit = "a8ffa60494a6e58e75e9286aca0eba170d5349b6",
    importpath = "github.com/getlantern/mockconn",
)

go_repository(
    name = "com_github_ghodss_yaml",
    commit = "0ca9ea5df5451ffdf184b4428c902747c2c11cd7",
    importpath = "github.com/ghodss/yaml",
)

go_repository(
    name = "com_github_go_logr_logr",
    commit = "9fb12b3b21c5415d16ac18dc5cd42c1cfdd40c4e",
    importpath = "github.com/go-logr/logr",
)

go_repository(
    name = "com_github_go_logr_zapr",
    commit = "7536572e8d55209135cd5e7ccf7fce43dca217ab",
    importpath = "github.com/go-logr/zapr",
)

go_repository(
    name = "com_github_go_openapi_jsonpointer",
    commit = "ef5f0afec364d3b9396b7b77b43dbe26bf1f8004",
    importpath = "github.com/go-openapi/jsonpointer",
)

go_repository(
    name = "com_github_go_openapi_jsonreference",
    commit = "8483a886a90412cd6858df4ea3483dce9c8e35a3",
    importpath = "github.com/go-openapi/jsonreference",
)

go_repository(
    name = "com_github_go_openapi_spec",
    commit = "5b6cdde3200976e3ecceb2868706ee39b6aff3e4",
    importpath = "github.com/go-openapi/spec",
)

go_repository(
    name = "com_github_go_openapi_swag",
    commit = "1d29f06aebd59ccdf11ae04aa0334ded96e2d909",
    importpath = "github.com/go-openapi/swag",
)

go_repository(
    name = "com_github_gobwas_glob",
    commit = "5ccd90ef52e1e632236f7326478d4faa74f99438",
    importpath = "github.com/gobwas/glob",
)

go_repository(
    name = "com_github_gogo_protobuf",
    build_file_proto_mode = "disable",
    commit = "1adfc126b41513cc696b209667c8656ea7aac67c",
    importpath = "github.com/gogo/protobuf",
)

go_repository(
    name = "com_github_golang_glog",
    commit = "23def4e6c14b4da8ac2ed8007337bc5eb5007998",
    importpath = "github.com/golang/glog",
)

go_repository(
    name = "com_github_golang_groupcache",
    commit = "5b532d6fd5efaf7fa130d4e859a2fde0fc3a9e1b",
    importpath = "github.com/golang/groupcache",
)

go_repository(
    name = "com_github_golang_mock",
    commit = "c34cdb4725f4c3844d095133c6e40e448b86589b",
    importpath = "github.com/golang/mock",
)

go_repository(
    name = "com_github_google_btree",
    commit = "e89373fe6b4a7413d7acd6da1725b83ef713e6e4",
    importpath = "github.com/google/btree",
)

go_repository(
    name = "com_github_google_go_cmp",
    commit = "5a6f75716e1203a923a78c9efb94089d857df0f6",
    importpath = "github.com/google/go-cmp",
)

go_repository(
    name = "com_github_google_gofuzz",
    commit = "24818f796faf91cd76ec7bddd72458fbced7a6c1",
    importpath = "github.com/google/gofuzz",
)

go_repository(
    name = "com_github_google_uuid",
    commit = "9b3b1e0f5f99ae461456d768e7d301a7acdaa2d8",
    importpath = "github.com/google/uuid",
)

go_repository(
    name = "com_github_googleapis_gax_go",
    commit = "317e0006254c44a0ac427cc52a0e083ff0b9622f",
    importpath = "github.com/googleapis/gax-go",
)

go_repository(
    name = "com_github_googleapis_gnostic",
    build_file_proto_mode = "disable",
    commit = "7c663266750e7d82587642f65e60bc4083f1f84e",
    importpath = "github.com/googleapis/gnostic",
)

go_repository(
    name = "com_github_gophercloud_gophercloud",
    commit = "4d3066f119fa80ac92c713ac747573df3e05ea28",
    importpath = "github.com/gophercloud/gophercloud",
)

go_repository(
    name = "com_github_gregjones_httpcache",
    commit = "9cad4c3443a7200dd6400aef47183728de563a38",
    importpath = "github.com/gregjones/httpcache",
)

go_repository(
    name = "com_github_grpc_ecosystem_go_grpc_prometheus",
    commit = "c225b8c3b01faf2899099b768856a9e916e5087b",
    importpath = "github.com/grpc-ecosystem/go-grpc-prometheus",
)

go_repository(
    name = "com_github_h2non_parth",
    commit = "b4df798d65426f8c8ab5ca5f9987aec5575d26c9",
    importpath = "github.com/h2non/parth",
)

go_repository(
    name = "com_github_hashicorp_golang_lru",
    commit = "20f1fb78b0740ba8c3cb143a61e86ba5c8669768",
    importpath = "github.com/hashicorp/golang-lru",
)

go_repository(
    name = "com_github_huandu_xstrings",
    commit = "f02667b379e2fb5916c3cda2cf31e0eb885d79f8",
    importpath = "github.com/huandu/xstrings",
)

go_repository(
    name = "com_github_imdario_mergo",
    commit = "9316a62528ac99aaecb4e47eadd6dc8aa6533d58",
    importpath = "github.com/imdario/mergo",
)

go_repository(
    name = "com_github_inconshreveable_mousetrap",
    commit = "76626ae9c91c4f2a10f34cad8ce83ea42c93bb75",
    importpath = "github.com/inconshreveable/mousetrap",
)

go_repository(
    name = "com_github_jhump_protoreflect",
    commit = "f90aa080ba36727321f2bc4c61f2a282f756bfab",
    importpath = "github.com/jhump/protoreflect",
)

go_repository(
    name = "com_github_json_iterator_go",
    commit = "ab8a2e0c74be9d3be70b3184d9acc634935ded82",
    importpath = "github.com/json-iterator/go",
)

go_repository(
    name = "com_github_konsorten_go_windows_terminal_sequences",
    commit = "edb144dfd453055e1e49a3d8b410a660b5a87613",
    importpath = "github.com/konsorten/go-windows-terminal-sequences",
)

go_repository(
    name = "com_github_mailru_easyjson",
    commit = "6243d8e04c3f819e79757e8bc3faa15c3cb27003",
    importpath = "github.com/mailru/easyjson",
)

go_repository(
    name = "com_github_makenowjust_heredoc",
    commit = "e9091a26100e9cfb2b6a8f470085bfa541931a91",
    importpath = "github.com/MakeNowJust/heredoc",
)

go_repository(
    name = "com_github_masterminds_goutils",
    commit = "41ac8693c5c10a92ea1ff5ac3a7f95646f6123b0",
    importpath = "github.com/Masterminds/goutils",
)

go_repository(
    name = "com_github_masterminds_semver",
    commit = "c7af12943936e8c39859482e61f0574c2fd7fc75",
    importpath = "github.com/Masterminds/semver",
)

go_repository(
    name = "com_github_masterminds_sprig",
    commit = "e4c0945838d570720d876a6ad9b4568ed32317b4",
    importpath = "github.com/Masterminds/sprig",
)

go_repository(
    name = "com_github_matttproud_golang_protobuf_extensions",
    commit = "c12348ce28de40eed0136aa2b644d0ee0650e56c",
    importpath = "github.com/matttproud/golang_protobuf_extensions",
)

go_repository(
    name = "com_github_mitchellh_copystructure",
    commit = "9a1b6f44e8da0e0e374624fb0a825a231b00c537",
    importpath = "github.com/mitchellh/copystructure",
)

go_repository(
    name = "com_github_mitchellh_go_wordwrap",
    commit = "9e67c67572bc5dd02aef930e2b0ae3c02a4b5a5c",
    importpath = "github.com/mitchellh/go-wordwrap",
)

go_repository(
    name = "com_github_mitchellh_reflectwalk",
    commit = "3e2c75dfad4fbf904b58782a80fd595c760ad185",
    importpath = "github.com/mitchellh/reflectwalk",
)

go_repository(
    name = "com_github_modern_go_concurrent",
    commit = "bacd9c7ef1dd9b15be4a9909b8ac7a4e313eec94",
    importpath = "github.com/modern-go/concurrent",
)

go_repository(
    name = "com_github_modern_go_reflect2",
    commit = "4b7aa43c6742a2c18fdef89dd197aaae7dac7ccd",
    importpath = "github.com/modern-go/reflect2",
)

go_repository(
    name = "com_github_motemen_go_loghttp",
    commit = "974ac5ceac271576caabe1c0bddf2b7eed471f67",
    importpath = "github.com/motemen/go-loghttp",
)

go_repository(
    name = "com_github_motemen_go_nuts",
    commit = "42c35bdb11c20ff50bb14ef05750e8dcdfc75ea5",
    importpath = "github.com/motemen/go-nuts",
)

go_repository(
    name = "com_github_onsi_gomega",
    commit = "62bff4df71bdbc266561a0caee19f0594b17c240",
    importpath = "github.com/onsi/gomega",
)

go_repository(
    name = "com_github_opencontainers_go_digest",
    commit = "279bed98673dd5bef374d3b6e4b09e2af76183bf",
    importpath = "github.com/opencontainers/go-digest",
)

go_repository(
    name = "com_github_pborman_uuid",
    commit = "adf5a7427709b9deb95d29d3fa8a2bf9cfd388f1",
    importpath = "github.com/pborman/uuid",
)

go_repository(
    name = "com_github_petar_gollrb",
    commit = "53be0d36a84c2a886ca057d34b6aa4468df9ccb4",
    importpath = "github.com/petar/GoLLRB",
)

go_repository(
    name = "com_github_peterbourgon_diskv",
    commit = "5f041e8faa004a95c88a202771f4cc3e991971e6",
    importpath = "github.com/peterbourgon/diskv",
)

go_repository(
    name = "com_github_pkg_errors",
    commit = "ba968bfe8b2f7e042a574c888954fccecfa385b4",
    importpath = "github.com/pkg/errors",
)

go_repository(
    name = "com_github_prometheus_client_golang",
    commit = "505eaef017263e299324067d40ca2c48f6a2cf50",
    importpath = "github.com/prometheus/client_golang",
)

go_repository(
    name = "com_github_prometheus_client_model",
    commit = "99fa1f4be8e564e8a6b613da7fa6f46c9edafc6c",
    importpath = "github.com/prometheus/client_model",
)

go_repository(
    name = "com_github_prometheus_common",
    commit = "7600349dcfe1abd18d72d3a1770870d9800a7801",
    importpath = "github.com/prometheus/common",
)

go_repository(
    name = "com_github_prometheus_procfs",
    commit = "7d6f385de8bea29190f15ba9931442a0eaef9af7",
    importpath = "github.com/prometheus/procfs",
)

go_repository(
    name = "com_github_puerkitobio_purell",
    commit = "44968752391892e1b0d0b821ee79e9a85fa13049",
    importpath = "github.com/PuerkitoBio/purell",
)

go_repository(
    name = "com_github_puerkitobio_urlesc",
    commit = "de5bf2ad457846296e2031421a34e2568e304e35",
    importpath = "github.com/PuerkitoBio/urlesc",
)

go_repository(
    name = "com_github_russross_blackfriday",
    commit = "d3b5b032dc8e8927d31a5071b56e14c89f045135",
    importpath = "github.com/russross/blackfriday",
)

go_repository(
    name = "com_github_shurcool_sanitized_anchor_name",
    commit = "7bfe4c7ecddb3666a94b053b422cdd8f5aaa3615",
    importpath = "github.com/shurcooL/sanitized_anchor_name",
)

go_repository(
    name = "com_github_sirupsen_logrus",
    commit = "60c74ad9be0d874af0ab0daef6ab07c5c5911f0d",
    importpath = "github.com/sirupsen/logrus",
)

go_repository(
    name = "com_github_spf13_cobra",
    commit = "ef82de70bb3f60c65fb8eebacbb2d122ef517385",
    importpath = "github.com/spf13/cobra",
)

go_repository(
    name = "com_github_spf13_pflag",
    commit = "583c0c0531f06d5278b7d917446061adc344b5cd",
    importpath = "github.com/spf13/pflag",
)

go_repository(
    name = "com_github_technosophos_moniker",
    commit = "a5dbd03a2245d554160e3ae6bfdcf969fe58b431",
    importpath = "github.com/technosophos/moniker",
)

go_repository(
    name = "com_google_cloud_go",
    commit = "777200caa7fb8936aed0f12b1fd79af64cc83ec9",
    importpath = "cloud.google.com/go",
)

go_repository(
    name = "in_gopkg_h2non_gock_v1",
    commit = "ba88c4862a27596539531ce469478a91bc5a0511",
    importpath = "gopkg.in/h2non/gock.v1",
)

go_repository(
    name = "in_gopkg_inf_v0",
    commit = "d2d2541c53f18d2a059457998ce2876cc8e67cbf",
    importpath = "gopkg.in/inf.v0",
)

go_repository(
    name = "in_gopkg_square_go_jose_v2",
    commit = "628223f44a71f715d2881ea69afc795a1e9c01be",
    importpath = "gopkg.in/square/go-jose.v2",
)

go_repository(
    name = "in_gopkg_yaml_v2",
    commit = "5420a8b6744d3b0345ab293f6fcba19c978f1183",
    importpath = "gopkg.in/yaml.v2",
)

go_repository(
    name = "in_gopkg_yaml_v3",
    commit = "a6ecf24a6d716d933bcbc255a2f5d492285b54f5",
    importpath = "gopkg.in/yaml.v3",
)

go_repository(
    name = "io_k8s_api",
    build_file_proto_mode = "disable",
    commit = "2034b8ee98796b95962f75a871718523ce607887",
    importpath = "k8s.io/api",
)

go_repository(
    name = "io_k8s_apiextensions_apiserver",
    build_extra_args = ["-exclude=vendor"],
    build_file_proto_mode = "disable",
    commit = "d98e7f9a1a1453e929a814d2d067f92776972da2",
    importpath = "k8s.io/apiextensions-apiserver",
)

go_repository(
    name = "io_k8s_apimachinery",
    build_file_proto_mode = "disable",
    commit = "657cd094317c09d73696c12abec7b97f840fa75e",
    importpath = "k8s.io/apimachinery",
)

go_repository(
    name = "io_k8s_apiserver",
    build_file_proto_mode = "disable",
    commit = "1ec86e4da56ce0573788fc12bb3a5530600c0e5d",
    importpath = "k8s.io/apiserver",
)

go_repository(
    name = "io_k8s_cli_runtime",
    commit = "c2e1885581d0f2b27818e3070f0b4dd4417c3493",
    importpath = "k8s.io/cli-runtime",
)

go_repository(
    name = "io_k8s_client_go",
    build_file_proto_mode = "disable",
    commit = "e6017390cea53c17205f14dc91694f14bf8db50a",
    importpath = "k8s.io/client-go",
)

go_repository(
    name = "io_k8s_helm",
    # v2.16.1 - Tue Nov 12 13:29:17 2019
    commit = "79d07943b03aea2b76c12644b4b54733bc5958d6",
    importpath = "k8s.io/helm",
)

go_repository(
    name = "io_k8s_klog",
    commit = "a5bc97fbc634d635061f3146511332c7e313a55a",
    importpath = "k8s.io/klog",
)

go_repository(
    name = "io_k8s_kube_openapi",
    build_file_proto_mode = "disable",
    commit = "b3a7cee44a305be0a69e1b9ac03018307287e1b0",
    importpath = "k8s.io/kube-openapi",
)

go_repository(
    name = "io_k8s_kubernetes",
    commit = "2166946f41b36dea2c4626f90a77706f426cdea2",
    importpath = "k8s.io/kubernetes",
)

go_repository(
    name = "io_k8s_sigs_controller_runtime",
    build_extra_args = ["-exclude=vendor"],
    commit = "9dc73700f6a8711204f8d4f12242c6b48fbcf7da",
    importpath = "sigs.k8s.io/controller-runtime",
)

go_repository(
    name = "io_k8s_sigs_kind",
    commit = "fd64a56b0c3d5654eb6d22bce812e2a87eac5853",
    importpath = "sigs.k8s.io/kind",
)

go_repository(
    name = "io_k8s_sigs_kustomize",
    commit = "a6f65144121d1955266b0cd836ce954c04122dc8",
    importpath = "sigs.k8s.io/kustomize",
)

go_repository(
    name = "io_k8s_sigs_yaml",
    commit = "fd68e9863619f6ec2fdd8625fe1f02e7c877e480",
    importpath = "sigs.k8s.io/yaml",
)

go_repository(
    name = "io_k8s_utils",
    commit = "581e00157fb1a0435d4fac54a52d1ca1e481d60e",
    importpath = "k8s.io/utils",
)

go_repository(
    name = "io_opencensus_go",
    commit = "e262766cd0d230a1bb7c37281e345e465f19b41b",
    importpath = "go.opencensus.io",
)

go_repository(
    name = "ml_vbom_util",
    commit = "efcd4e0f97874370259c7d93e12aad57911dea81",
    importpath = "vbom.ml/util",
)

go_repository(
    name = "org_golang_google_api",
    commit = "7c8bab9c4f5dd00b2afd0648fa0d753a46a603e8",
    importpath = "google.golang.org/api",
)

go_repository(
    name = "org_golang_google_appengine",
    commit = "b1f26356af11148e710935ed1ac8a7f5702c7612",
    importpath = "google.golang.org/appengine",
)

go_repository(
    name = "org_golang_google_genproto",
    commit = "ff3583edef7de132f219f0efc00e097cabcc0ec0",
    importpath = "google.golang.org/genproto",
)

go_repository(
    name = "org_golang_google_grpc",
    commit = "754ee590a4f386d0910d887f3b8776354042260b",
    importpath = "google.golang.org/grpc",
)

go_repository(
    name = "org_golang_x_crypto",
    commit = "a49355c7e3f8fe157a85be2f77e6e269a0f89602",
    importpath = "golang.org/x/crypto",
)

go_repository(
    name = "org_golang_x_net",
    commit = "5f4716e94777e714bc2fb3e3a44599cb40817aac",
    importpath = "golang.org/x/net",
)

go_repository(
    name = "org_golang_x_oauth2",
    commit = "ef147856a6ddbb60760db74283d2424e98c87bff",
    importpath = "golang.org/x/oauth2",
)

go_repository(
    name = "org_golang_x_sync",
    commit = "1d60e4601c6fd243af51cc01ddf169918a5407ca",
    importpath = "golang.org/x/sync",
)

go_repository(
    name = "org_golang_x_sys",
    commit = "7138fd3d9dc8335c567ca206f4333fb75eb05d56",
    importpath = "golang.org/x/sys",
)

go_repository(
    name = "org_golang_x_text",
    commit = "f21a4dfb5e38f5895301dc265a8def02365cc3d0",
    importpath = "golang.org/x/text",
)

go_repository(
    name = "org_golang_x_time",
    commit = "fbb02b2291d28baffd63558aa44b4b56f178d650",
    importpath = "golang.org/x/time",
)

go_repository(
    name = "org_uber_go_atomic",
    commit = "1ea20fb1cbb1cc08cbd0d913a96dead89aa18289",
    importpath = "go.uber.org/atomic",
)

go_repository(
    name = "org_uber_go_multierr",
    commit = "3c4937480c32f4c13a875a1829af76c98ca3d40a",
    importpath = "go.uber.org/multierr",
)

go_repository(
    name = "org_uber_go_zap",
    commit = "ff33455a0e382e8a81d14dd7c922020b6b5e7982",
    importpath = "go.uber.org/zap",
)

go_repository(
    name = "com_github_asaskevich_govalidator",
    commit = "ccb8e960c48f04d6935e72476ae4a51028f9e22f",
    importpath = "github.com/asaskevich/govalidator",
)

go_repository(
    name = "com_github_coreos_etcd",
    build_file_proto_mode = "disable",
    commit = "98d308426819d892e149fe45f6fd542464cb1f9d",
    importpath = "github.com/coreos/etcd",
)

go_repository(
    name = "com_github_coreos_go_systemd",
    commit = "95778dfbb74eb7e4dbaf43bf7d71809650ef8076",
    importpath = "github.com/coreos/go-systemd",
)

go_repository(
    name = "com_github_globalsign_mgo",
    commit = "eeefdecb41b842af6dc652aaea4026e8403e62df",
    importpath = "github.com/globalsign/mgo",
)

go_repository(
    name = "com_github_go_openapi_analysis",
    commit = "e2f3fdbb7ed0e56e070ccbfb6fc75b288a33c014",
    importpath = "github.com/go-openapi/analysis",
)

go_repository(
    name = "com_github_go_openapi_errors",
    commit = "7a7ff1b7b8020f22574411a32f28b4d168d69237",
    importpath = "github.com/go-openapi/errors",
)

go_repository(
    name = "com_github_go_openapi_loads",
    commit = "74628589c3b94e3526a842d24f46589980f5ab22",
    importpath = "github.com/go-openapi/loads",
)

go_repository(
    name = "com_github_go_openapi_runtime",
    commit = "109737172424d8a656fd1199e28c9f5cc89b0cca",
    importpath = "github.com/go-openapi/runtime",
)

go_repository(
    name = "com_github_go_openapi_strfmt",
    commit = "29177d4b5db488583bb97ebc05d3842ebeda91a8",
    importpath = "github.com/go-openapi/strfmt",
)

go_repository(
    name = "com_github_go_openapi_validate",
    commit = "5b1623be7460f5a3967a82c00d518048fb190f5e",
    importpath = "github.com/go-openapi/validate",
)

go_repository(
    name = "com_github_hashicorp_yamux",
    commit = "2f1d1f20f75d5404f53b9edf6b53ed5505508675",
    importpath = "github.com/hashicorp/yamux",
)

go_repository(
    name = "com_github_mattn_go_isatty",
    commit = "7b513a986450394f7bbf1476909911b3aa3a55ce",
    importpath = "github.com/mattn/go-isatty",
)

go_repository(
    name = "com_github_mitchellh_mapstructure",
    commit = "3536a929edddb9a5b34bd6861dc4a9647cb459fe",
    importpath = "github.com/mitchellh/mapstructure",
)

go_repository(
    name = "com_github_nytimes_gziphandler",
    commit = "dd0439581c7657cb652dfe5c71d7d48baf39541d",
    importpath = "github.com/NYTimes/gziphandler",
)

go_repository(
    name = "in_gopkg_natefinch_lumberjack_v2",
    commit = "a96e63847dc3c67d17befa69c303767e2f84e54f",
    importpath = "gopkg.in/natefinch/lumberjack.v2",
)

go_repository(
    name = "com_github_munnerz_goautoneg",
    commit = "2ae31c8b6b30d2f4c8100c20d527b571e9c433bb",
    importpath = "github.com/munnerz/goautoneg",
)

go_repository(
    name = "com_github_pelletier_go_toml",
    commit = "903d9455db9ff1d7ac1ab199062eca7266dd11a3",
    importpath = "github.com/pelletier/go-toml",
)

go_repository(
    name = "io_k8s_component_base",
    commit = "2354f2393ad4e816a2996f727ee030abe4e34727",
    importpath = "k8s.io/component-base",
)

go_repository(
    name = "io_k8s_sigs_structured_merge_diff",
    commit = "e85c7b244fd2cc57bb829d73a061f93a441e63ce",
    importpath = "sigs.k8s.io/structured-merge-diff",
)

go_repository(
    name = "xyz_gomodules_jsonpatch",
    commit = "e8422f09d27ee2c8cfb2c7f8089eb9eeb0764849",
    importpath = "gomodules.xyz/jsonpatch",
    remote = "https://github.com/gomodules/jsonpatch.git",
    vcs = "git",
)
# End of "Generated by gazelle".

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

go_repository(
    name = "com_github_liggitt_tabwriter",
    commit = "89fcab3d43de07060e4fd4c1547430ed57e87f24",
    importpath = "github.com/liggitt/tabwriter",
)

go_repository(
    name = "org_golang_google_protobuf",
    commit = "3f7a61f89bb6813f89d981d1870ed68da0b3c3f1",
    importpath = "google.golang.org/protobuf",
)
