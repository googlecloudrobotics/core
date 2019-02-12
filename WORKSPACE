workspace(name = "cloud_robotics")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")
load("//bazel:repositories.bzl", "cloud_robotics_repositories")

cloud_robotics_repositories()

# Protobuf
http_archive(
    name = "com_google_protobuf",
    sha256 = "2244b0308846bb22b4ff0bcc675e99290ff9f1115553ae9671eba1030af31bc0",
    strip_prefix = "protobuf-3.6.1.2",
    urls = [
        "https://mirror.bazel.build/github.com/google/protobuf/archive/v3.6.1.2.tar.gz",
        "https://github.com/google/protobuf/archive/v3.6.1.2.tar.gz",
    ],
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
    "@io_bazel_rules_docker//container:container.bzl",
    "container_pull",
    container_repositories = "repositories",
)

container_repositories()

container_pull(
    name = "iptables_base",
    digest = "sha256:cd81b1a8f40149b5061735927d2a2cf4b90fc27a52fc4cc66889b373368b6ef6",
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

load(
    "@io_bazel_rules_docker//python:image.bzl",
    _py_image_repos = "repositories",
)

_py_image_repos()

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

# Bazel build tools (must be before go_rules_dependencies() to take priority)
http_archive(
    name = "com_github_bazelbuild_buildtools",
    sha256 = "d42e4c9727958bc5814d3bc44f19db5a24f419436cbba09f1e8913eb4a09da31",
    strip_prefix = "buildtools-0.19.2.1",
    urls = [
        "https://github.com/bazelbuild/buildtools/archive/0.19.2.1.tar.gz",
    ],
)

load("@com_github_bazelbuild_buildtools//buildifier:deps.bzl", "buildifier_dependencies")

buildifier_dependencies()

# Go rules and proto support
http_archive(
    name = "io_bazel_rules_go",
    sha256 = "8be57ff66da79d9e4bd434c860dce589195b9101b2c187d144014bbca23b5166",
    strip_prefix = "rules_go-0.16.3",
    urls = [
        "https://github.com/bazelbuild/rules_go/archive/0.16.3.tar.gz",
    ],
)

http_archive(
    name = "bazel_gazelle",
    sha256 = "a2c6b31cd295648779a92d5b8a255da4494e95c7383afd5058334cdce6a80d10",
    strip_prefix = "bazel-gazelle-455e69320ee92c6f3bfb267aa211a4fa6ebc4e5d",
    urls = [
        "https://github.com/bazelbuild/bazel-gazelle/archive/455e69320ee92c6f3bfb267aa211a4fa6ebc4e5d.tar.gz",
    ],
)

load("@io_bazel_rules_go//go:def.bzl", "go_register_toolchains")
load("@bazel_gazelle//:def.bzl", "go_repository")

# Do not call go_rules_dependencies() until after all other go_repository
# calls, or else the top-level definitions may be silently ignored.
# https://github.com/bazelbuild/bazel/issues/3908
go_register_toolchains()

# Third party Go libraries
go_repository(
    name = "com_github_PuerkitoBio_purell",
    commit = "8a290539e2e8629dbc4e6bad948158f790ec31f4",
    importpath = "github.com/PuerkitoBio/purell",
)

go_repository(
    name = "com_github_PuerkitoBio_urlesc",
    commit = "5bd2802263f21d8788851d5305584c82a5c75d7e",
    importpath = "github.com/PuerkitoBio/urlesc",
)

go_repository(
    name = "com_github_puerkitobio_purell",
    commit = "0bcb03f4b4d0a9428594752bd2a3b9aa0a9d4bd4",
    importpath = "github.com/PuerkitoBio/purell",
)

go_repository(
    name = "com_github_puerkitobio_urlesc",
    commit = "de5bf2ad457846296e2031421a34e2568e304e35",
    importpath = "github.com/PuerkitoBio/urlesc",
)

go_repository(
    name = "com_github_davecgh_go_spew",
    commit = "8991bc29aa16c548c550c7ff78260e27b9ab7c73",
    importpath = "github.com/davecgh/go-spew",
)

go_repository(
    name = "com_github_emicklei_go_restful",
    commit = "3eb9738c1697594ea6e71a7156a9bb32ed216cf0",
    importpath = "github.com/emicklei/go-restful",
)

go_repository(
    name = "com_github_ghodss_yaml",
    commit = "0ca9ea5df5451ffdf184b4428c902747c2c11cd7",
    importpath = "github.com/ghodss/yaml",
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
    name = "com_github_gogo_protobuf",
    commit = "1adfc126b41513cc696b209667c8656ea7aac67c",
    importpath = "github.com/gogo/protobuf",
)

go_repository(
    name = "com_github_golang_glog",
    commit = "23def4e6c14b4da8ac2ed8007337bc5eb5007998",
    importpath = "github.com/golang/glog",
)

go_repository(
    name = "com_github_golang_mock",
    commit = "c34cdb4725f4c3844d095133c6e40e448b86589b",
    importpath = "github.com/golang/mock",
)

go_repository(
    name = "bazel_gomock",
    commit = "38c805849d06a1dd67edb30576f963eb56e9154a",
    importpath = "github.com/jmhodges/bazel_gomock",
)

go_repository(
    name = "com_github_google_gofuzz",
    commit = "24818f796faf91cd76ec7bddd72458fbced7a6c1",
    importpath = "github.com/google/gofuzz",
)

go_repository(
    name = "com_github_googleapis_gax_go",
    commit = "317e0006254c44a0ac427cc52a0e083ff0b9622f",
    importpath = "github.com/googleapis/gax-go",
)

go_repository(
    name = "com_github_hashicorp_golang_lru",
    commit = "20f1fb78b0740ba8c3cb143a61e86ba5c8669768",
    importpath = "github.com/hashicorp/golang-lru",
)

go_repository(
    name = "com_github_jhump_protoreflect",
    commit = "d0f5a643a7920db0ccead732682f4053fbc32834",
    importpath = "github.com/jhump/protoreflect",
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
    name = "com_github_mailru_easyjson",
    commit = "60711f1a8329503b04e1c88535f419d0bb440bff",
    importpath = "github.com/mailru/easyjson",
)

go_repository(
    name = "com_github_ryanuber_go_license",
    commit = "3878fc8f33ea4637cf02cf8a0bf926288a246ceb",
    importpath = "github.com/ryanuber/go-license",
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
    name = "com_google_cloud_go",
    commit = "777200caa7fb8936aed0f12b1fd79af64cc83ec9",
    importpath = "cloud.google.com/go",
)

go_repository(
    name = "in_gopkg_inf_v0",
    commit = "d2d2541c53f18d2a059457998ce2876cc8e67cbf",
    importpath = "gopkg.in/inf.v0",
)

go_repository(
    name = "in_gopkg_yaml_v2",
    commit = "5420a8b6744d3b0345ab293f6fcba19c978f1183",
    importpath = "gopkg.in/yaml.v2",
)

go_repository(
    name = "io_k8s_apimachinery",
    build_file_proto_mode = "disable",
    commit = "2b1284ed4c93a43499e781493253e2ac5959c4fd",
    importpath = "k8s.io/apimachinery",
)

go_repository(
    name = "io_k8s_client_go",
    build_file_proto_mode = "disable",
    commit = "6bf63545bd0257ed9e701ad95307ffa51b4407c0",
    importpath = "k8s.io/client-go",
)

go_repository(
    name = "io_k8s_apiextensions_apiserver",
    # Ignore the vendored k8s deps, since we pull in compatible versions in WORKSPACE.
    build_extra_args = ["-exclude=vendor/k8s.io"],
    build_file_proto_mode = "disable",
    commit = "7d26de67f177df719a36756e658973478df68485",
    importpath = "k8s.io/apiextensions-apiserver",
)

go_repository(
    name = "io_k8s_gengo",
    build_file_proto_mode = "disable",
    commit = "fdcf9f9480fdd5bf2b3c3df9bf4ecd22b25b87e2",
    importpath = "k8s.io/gengo",
)

go_repository(
    name = "io_k8s_kube_openapi",
    build_file_proto_mode = "disable",
    commit = "d83b052f768a50a309c692a9c271da3f3276ff88",
    importpath = "k8s.io/kube-openapi",
)

go_repository(
    name = "org_golang_google_api",
    commit = "7c8bab9c4f5dd00b2afd0648fa0d753a46a603e8",
    importpath = "google.golang.org/api",
)

go_repository(
    name = "org_golang_google_genproto",
    commit = "ff3583edef7de132f219f0efc00e097cabcc0ec0",
    importpath = "google.golang.org/genproto",
)

go_repository(
    name = "org_golang_google_grpc",
    commit = "168a6198bcb0ef175f7dacec0b8691fc141dc9b8",
    importpath = "google.golang.org/grpc",
)

go_repository(
    name = "org_golang_x_crypto",
    commit = "a49355c7e3f8fe157a85be2f77e6e269a0f89602",
    importpath = "golang.org/x/crypto",
)

go_repository(
    name = "org_golang_x_net",
    commit = "4cb1c02c05b0e749b0365f61ae859a8e0cfceed9",
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
    name = "github_com_h2non_gock",
    commit = "4692cba4394f79372bfe85996501ad43db9d82e0",
    importpath = "github.com/h2non/gock",
)

go_repository(
    name = "grpc_ecosystem_grpc_gateway",
    commit = "d8ad87ee91e1062c2a6117bda0b5523e9cb949ef",
    importpath = "github.com/grpc-ecosystem/grpc-gateway",
)

go_repository(
    name = "com_github_google_btree",
    commit = "e89373fe6b4a7413d7acd6da1725b83ef713e6e4",
    importpath = "github.com/google/btree",
)

go_repository(
    name = "com_github_googleapis_gnostic",
    build_file_proto_mode = "disable",
    commit = "7c663266750e7d82587642f65e60bc4083f1f84e",
    importpath = "github.com/googleapis/gnostic",
)

go_repository(
    name = "com_github_gregjones_httpcache",
    commit = "9cad4c3443a7200dd6400aef47183728de563a38",
    importpath = "github.com/gregjones/httpcache",
)

go_repository(
    name = "com_github_json_iterator_go",
    commit = "ab8a2e0c74be9d3be70b3184d9acc634935ded82",
    importpath = "github.com/json-iterator/go",
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
    name = "io_k8s_api",
    build_file_proto_mode = "disable",
    commit = "67edc246be36579e46a89e29a2f165d47e012109",
    importpath = "k8s.io/api",
)

go_repository(
    name = "org_golang_google_appengine",
    commit = "b1f26356af11148e710935ed1ac8a7f5702c7612",
    importpath = "google.golang.org/appengine",
)

go_repository(
    name = "com_github_beorn7_perks",
    commit = "3a771d992973f24aa725d07868b467d1ddfceafb",
    importpath = "github.com/beorn7/perks",
)

go_repository(
    name = "com_github_matttproud_golang_protobuf_extensions",
    commit = "c12348ce28de40eed0136aa2b644d0ee0650e56c",
    importpath = "github.com/matttproud/golang_protobuf_extensions",
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
    name = "com_github_prometheus_alertmanager",
    commit = "d19fae3bae451940b8470abb680cfdd59bfa7cfa",
    importpath = "github.com/prometheus/alertmanager",
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
    name = "io_opencensus_go",
    commit = "6edeb78af2d9e4f169abb223feaef35da2e45d06",
    importpath = "go.opencensus.io",
)

go_repository(
    name = "com_github_kubernetes_sigs_kubebuilder",
    commit = "a523fb6b174c2785084eb0948c4cfdaf2f818bc4",
    importpath = "github.com/kubernetes-sigs/kubebuilder",
)

go_repository(
    name = "com_github_onsi_ginkgo",
    commit = "fa5fabab2a1bfbd924faf4c067d07ae414e2aedf",
    importpath = "github.com/onsi/ginkgo",
)

go_repository(
    name = "com_github_onsi_gomega",
    commit = "62bff4df71bdbc266561a0caee19f0594b17c240",
    importpath = "github.com/onsi/gomega",
)

go_repository(
    name = "io_k8s_kube_aggregator",
    commit = "ec87e2e7ece3dcdad53940e14227d75c1f19843f",
    importpath = "k8s.io/kube-aggregator",
)

go_repository(
    name = "io_k8s_sigs_testing_frameworks",
    commit = "f53464b8b84b4507805a0b033a8377b225163fea",
    importpath = "sigs.k8s.io/testing_frameworks",
)

go_repository(
    name = "com_github_motemen_go_nuts",
    commit = "42c35bdb11c20ff50bb14ef05750e8dcdfc75ea5",
    importpath = "github.com/motemen/go-nuts",
)

go_repository(
    name = "com_github_motemen_go_loghttp",
    commit = "974ac5ceac271576caabe1c0bddf2b7eed471f67",
    importpath = "github.com/motemen/go-loghttp",
)

go_repository(
    name = "com_github_fsnotify_fsnotify",
    commit = "c2828203cd70a50dcccfb2761f8b1f8ceef9a8e9",
    importpath = "github.com/fsnotify/fsnotify",
)

go_repository(
    name = "io_k8s_helm",
    commit = "eecf22f77df5f65c823aacd2dbd30ae6c65f186e",
    importpath = "k8s.io/helm",
)

go_repository(
    name = "com_github_evanphx_json_patch",
    commit = "72bf35d0ff611848c1dc9df0f976c81192392fa5",
    importpath = "github.com/evanphx/json-patch",
)

go_repository(
    name = "io_k8s_klog",
    commit = "a5bc97fbc634d635061f3146511332c7e313a55a",
    importpath = "k8s.io/klog",
)

go_repository(
    name = "io_k8s_sigs_yaml",
    commit = "fd68e9863619f6ec2fdd8625fe1f02e7c877e480",
    importpath = "sigs.k8s.io/yaml",
)

go_repository(
    name = "com_github_appscode_jsonpatch",
    commit = "7c0e3b262f30165a8ec3d0b4c6059fd92703bfb2",
    importpath = "github.com/appscode/jsonpatch",
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
    name = "com_github_golang_groupcache",
    commit = "5b532d6fd5efaf7fa130d4e859a2fde0fc3a9e1b",
    importpath = "github.com/golang/groupcache",
)

go_repository(
    name = "com_github_google_uuid",
    commit = "9b3b1e0f5f99ae461456d768e7d301a7acdaa2d8",
    importpath = "github.com/google/uuid",
)

go_repository(
    name = "com_github_pborman_uuid",
    commit = "adf5a7427709b9deb95d29d3fa8a2bf9cfd388f1",
    importpath = "github.com/pborman/uuid",
)

go_repository(
    name = "io_k8s_sigs_controller_runtime",
    build_extra_args = ["-exclude=vendor"],
    commit = "12d98582e72927b6cd0123e2b4e819f9341ce62c",
    importpath = "sigs.k8s.io/controller-runtime",
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
    name = "com_github_aokoli_goutils",
    commit = "41ac8693c5c10a92ea1ff5ac3a7f95646f6123b0",
    importpath = "github.com/aokoli/goutils",
)

go_repository(
    name = "com_github_burntsushi_toml",
    commit = "3012a1dbe2e4bd1391d42b32f0577cb7bbc7f005",
    importpath = "github.com/BurntSushi/toml",
)

go_repository(
    name = "com_github_gobwas_glob",
    commit = "5ccd90ef52e1e632236f7326478d4faa74f99438",
    importpath = "github.com/gobwas/glob",
)

go_repository(
    name = "com_github_huandu_xstrings",
    commit = "f02667b379e2fb5916c3cda2cf31e0eb885d79f8",
    importpath = "github.com/huandu/xstrings",
)

go_repository(
    name = "com_github_masterminds_semver",
    commit = "c7af12943936e8c39859482e61f0574c2fd7fc75",
    importpath = "github.com/Masterminds/semver",
)

go_repository(
    name = "com_github_masterminds_sprig",
    commit = "544a9b1d90f323f6509491b389714fbbd126bee3",
    importpath = "github.com/Masterminds/sprig",
)

go_repository(
    name = "com_github_azure_go_ansiterm",
    commit = "d6e3b3328b783f23731bc4d058875b0371ff8109",
    importpath = "github.com/Azure/go-ansiterm",
)

go_repository(
    name = "com_github_chai2010_gettext_go",
    commit = "bf70f2a70fb1b1f36d90d671a72795984eab0fcb",
    importpath = "github.com/chai2010/gettext-go",
)

go_repository(
    name = "com_github_docker_distribution",
    commit = "2461543d988979529609e8cb6fca9ca190dc48da",
    importpath = "github.com/docker/distribution",
)

go_repository(
    name = "com_github_docker_docker",
    commit = "092cba3727bb9b4a2f0e922cd6c0f93ea270e363",
    importpath = "github.com/docker/docker",
)

go_repository(
    name = "com_github_docker_go_connections",
    commit = "7395e3f8aa162843a74ed6d48e79627d9792ac55",
    importpath = "github.com/docker/go-connections",
)

go_repository(
    name = "com_github_docker_go_units",
    commit = "47565b4f722fb6ceae66b95f853feed578a4a51c",
    importpath = "github.com/docker/go-units",
)

go_repository(
    name = "com_github_docker_spdystream",
    commit = "6480d4af844c189cf5dd913db24ddd339d3a4f85",
    importpath = "github.com/docker/spdystream",
)

go_repository(
    name = "com_github_exponent_io_jsonpath",
    commit = "d6023ce2651d8eafb5c75bb0c7167536102ec9f5",
    importpath = "github.com/exponent-io/jsonpath",
)

go_repository(
    name = "com_github_fatih_camelcase",
    commit = "44e46d280b43ec1531bb25252440e34f1b800b65",
    importpath = "github.com/fatih/camelcase",
)

go_repository(
    name = "com_github_konsorten_go_windows_terminal_sequences",
    commit = "5c8c8bd35d3832f5d134ae1e1e375b69a4d25242",
    importpath = "github.com/konsorten/go-windows-terminal-sequences",
)

go_repository(
    name = "com_github_makenowjust_heredoc",
    commit = "e9091a26100e9cfb2b6a8f470085bfa541931a91",
    importpath = "github.com/MakeNowJust/heredoc",
)

go_repository(
    name = "com_github_mitchellh_go_wordwrap",
    commit = "9e67c67572bc5dd02aef930e2b0ae3c02a4b5a5c",
    importpath = "github.com/mitchellh/go-wordwrap",
)

go_repository(
    name = "com_github_opencontainers_go_digest",
    commit = "279bed98673dd5bef374d3b6e4b09e2af76183bf",
    importpath = "github.com/opencontainers/go-digest",
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
    commit = "e1e72e9de974bd926e5c56f83753fba2df402ce5",
    importpath = "github.com/Sirupsen/logrus",
)

go_repository(
    name = "in_gopkg_square_go_jose_v2",
    commit = "e94fb177d3668d35ab39c61cbb2f311550557e83",
    importpath = "gopkg.in/square/go-jose.v2",
)

go_repository(
    name = "io_k8s_apiserver",
    commit = "d50e9ac5404fc1ac6d053e85740360e8295c0865",
    importpath = "k8s.io/apiserver",
)

go_repository(
    name = "io_k8s_kubernetes",
    commit = "65ecaf0671341311ce6aea0edab46ee69f65d59e",
    importpath = "k8s.io/kubernetes",
)

go_repository(
    name = "io_k8s_utils",
    commit = "4ae6e769426ee1a06523eb30fcb69c0a01f03772",
    importpath = "k8s.io/utils",
)

go_repository(
    name = "ml_vbom_util",
    commit = "efcd4e0f97874370259c7d93e12aad57911dea81",
    importpath = "vbom.ml/util",
)

go_repository(
    name = "com_github_cyphar_filepath_securejoin",
    commit = "a261ee33d7a517f054effbf451841abaafe3e0fd",
    importpath = "github.com/cyphar/filepath-securejoin",
)

go_repository(
    name = "com_github_pkg_errors",
    commit = "ba968bfe8b2f7e042a574c888954fccecfa385b4",
    importpath = "github.com/pkg/errors",
)

http_archive(
    name = "com_github_kubernetes_sigs_application",
    build_file = "@cloud_robotics//third_party:app_crd.BUILD",
    sha256 = "50cd3e3427355f77d177f7cee7704b1f077a2564e67d07db762bdb3022c6cb58",
    strip_prefix = "application-464aad9c212082e99a9f9ed4515f1bd1f1df2bfa",
    urls = [
        "https://github.com/kubernetes-sigs/application/archive/464aad9c212082e99a9f9ed4515f1bd1f1df2bfa.tar.gz",
    ],
)

load("@io_bazel_rules_go//go:def.bzl", "go_rules_dependencies")

go_rules_dependencies()

load("@bazel_gazelle//:deps.bzl", "gazelle_dependencies")

gazelle_dependencies()

# Containerization rules for Go must come after go_rules_dependencies().
load(
    "@io_bazel_rules_docker//go:image.bzl",
    _go_image_repos = "repositories",
)

_go_image_repos()

# Add Maven dependencies
load("//third_party:maven_dependencies.bzl", "maven_dependencies")

maven_dependencies()

# rules_python
# TODO(rodrigoq): revert back to upstream once this issue is resolved:
# https://github.com/bazelbuild/rules_python/issues/14
http_archive(
    name = "io_bazel_rules_python",
    sha256 = "71990f9dbdd9817607e1427889e3db01e04a0840841ac14d9c21df6d0940cf2a",
    strip_prefix = "rules_python-e092647bd5d33858041e61f018e19f4026c6fc46",
    urls = [
        "https://github.com/drigz/rules_python/archive/e092647bd5d33858041e61f018e19f4026c6fc46.tar.gz",
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

# Helm subcharts
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_file")

http_file(
    name = "prometheus_node_exporter_chart",
    sha256 = "",
    urls = [
        "https://storage.googleapis.com/kubernetes-charts/prometheus-node-exporter-0.5.0.tgz",
    ],
)
