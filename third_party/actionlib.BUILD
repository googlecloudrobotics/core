# Description:
#   A standardized interface for interfacing with preemptable tasks.

load("@cloud_robotics//bazel/build_rules:rosmsg.bzl", "rosmsg")

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])  # BSD

cc_library(
    name = "actionlib",
    srcs = glob(["src/*.cpp"]),
    hdrs = glob(["include/**/*.h"]),
    includes = ["include"],
    deps = [
        "@boost//:bind",
        "@boost//:concept_check",
        "@boost//:interprocess",
        "@boost//:scoped_ptr",
        "@boost//:shared_ptr",
        "@boost//:smart_ptr",
        "@boost//:thread",
        "@com_github_ros_common_msgs//:actionlib_msgs_cpp",
        "@com_github_ros_ros_comm//:rosconsole",
        "@com_github_ros_ros_comm//:roscpp_lib",
        "@com_github_ros_roscpp_core//:cpp_common",
        "@com_github_ros_roscpp_core//:rostime",
    ],
)
