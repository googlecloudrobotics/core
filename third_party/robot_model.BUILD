# Description:
#   Packages for modeling various aspects of robot information.

load("@bazel_rules//:config.bzl", "cc_fix_config")

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])  # BSD

# TODO(rodrigoq): extract version from urdfdom package
cc_fix_config(
    name = "urdfdom_compatibility_h",
    cmake = True,
    files = {"urdf/urdfdom_compatibility.h.in": "urdf/urdfdom_compatibility.h"},
    values = {
        "URDFDOM_HEADERS_MAJOR_VERSION": "1",
        "URDFDOM_HEADERS_MINOR_VERSION": "0",
        "URDFDOM_HEADERS_REVISION_VERSION": "0",
    },
    visibility = ["//visibility:private"],
)

cc_library(
    name = "urdfdom_compatibility",
    hdrs = ["urdf/urdfdom_compatibility.h"],
    includes = ["."],
)

cc_library(
    name = "urdf_parser_plugin",
    hdrs = ["urdf_parser_plugin/include/urdf_parser_plugin/parser.h"],
    includes = ["urdf_parser_plugin/include"],
    deps = [":urdfdom_compatibility"],
)

cc_library(
    name = "urdf",
    srcs = ["urdf/src/model.cpp"],
    hdrs = ["urdf/include/urdf/model.h"],
    includes = ["urdf/include"],
    deps = [
        ":urdf_parser_plugin",
        ":urdfdom_compatibility",
        "@com_github_icebreaker_tinyxml//:tinyxml",
        "@com_github_ros_pluginlib//:pluginlib",
        "@com_github_ros_ros_comm//:roscpp_lib",
        "@com_github_ros_urdfdom//:urdf_parser",
        "@com_github_ros_urdfdom_headers//:urdf_model",
        "@com_github_ros_urdfdom_headers//:urdf_world",
    ],
)
