load("@io_bazel_rules_go//go:def.bzl", "go_binary", "go_test")

go_binary(
    name = "relay_load_test",
    srcs = ["relay_load_test.go"],
    deps = [
        "//src/go/cmd/http-relay-client/client:go_default_library",
        "//src/go/cmd/http-relay-server/server:go_default_library",
    ],
)
