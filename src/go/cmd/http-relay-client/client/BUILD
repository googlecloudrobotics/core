load("@io_bazel_rules_go//go:def.bzl", "go_library", "go_test")

package(default_visibility = ["//visibility:public"])

go_library(
    name = "go_default_library",
    srcs = [
        "client.go",
    ],
    importpath = "github.com/googlecloudrobotics/core/src/go/cmd/http-relay-client/client",
    deps = [
        "//src/proto/http-relay:go_default_library",
        "@com_github_cenkalti_backoff//:go_default_library",
        "@com_github_golang_glog//:go_default_library",
        "@io_opencensus_go//plugin/ochttp:go_default_library",
        "@io_opencensus_go//plugin/ochttp/propagation/tracecontext:go_default_library",
        "@io_opencensus_go//trace:go_default_library",
        "@io_opencensus_go_contrib_exporter_stackdriver//:go_default_library",
        "@org_golang_google_protobuf//proto:go_default_library",
        "@org_golang_x_net//context:go_default_library",
        "@org_golang_x_net//http2:go_default_library",
        "@org_golang_x_oauth2//google:go_default_library",
    ],
)

go_test(
    name = "go_default_test",
    srcs = [
        "client_test.go",
    ],
    embed = [":go_default_library"],
    visibility = ["//visibility:private"],
    deps = [
        "//src/proto/http-relay:go_default_library",
        "@com_github_getlantern_httptest//:go_default_library",
        "@com_github_onsi_gomega//:go_default_library",
        "@in_gopkg_h2non_gock_v1//:go_default_library",
        "@org_golang_google_protobuf//proto:go_default_library",
    ],
)
