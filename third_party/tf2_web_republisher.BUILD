# Description:
#   Republishing of Selected TFs
#   http://robotwebtools.org/
#
load("@cloud_robotics//bazel/build_rules:rosmsg.bzl", "rosmsg")
load("@ros_deps//:requirements.bzl", "requirement")

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])  # BSD

rosmsg(
    name = "msgs",
    srcs = glob([
        "msg/*.msg",
        "services/*.srv",
        "action/*.action",
    ]),
    data = ["package.xml"],
    rospkg = "tf2_web_republisher",
    deps = [
        "@com_github_ros_common_msgs//:actionlib_msgs",
        "@com_github_ros_common_msgs//:geometry_msgs",
    ],
)

cc_library(
    name = "tf_pair",
    hdrs = ["include/tf_pair.h"],
    includes = ["include"],
    deps = [
        "@com_github_ros_geometry//:tf_lib",
    ],
)

cc_binary(
    name = "tf2_web_republisher",
    srcs = ["src/tf_web_republisher.cpp"],
    copts = ["-Wno-reorder"],
    data = ["package.xml"],
    linkstatic = 1,
    deps = [
        ":msgs_cpp",
        ":tf_pair",
        "@boost//:thread",
        "@com_github_ros_actionlib//:actionlib",
        "@com_github_ros_common_msgs//:geometry_msgs_cpp",
        "@com_github_ros_geometry//:tf_lib",
        "@com_github_ros_geometry2//:tf2_ros",
        "@com_github_ros_ros_comm//:roscpp_lib",
    ],
)
