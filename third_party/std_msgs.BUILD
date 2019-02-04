# Description:
#   std_msgs contains common message types representing primitive data types
#   and other basic message constructs, such as multiarrays. For common,
#   generic robot-specific message types, please see common_msgs.

load("@cloud_robotics//bazel/build_rules:rosmsg.bzl", "rosmsg")

licenses(["notice"])  # BSD

package(
    default_visibility = ["//visibility:public"],
)

cc_library(
    name = "builtins",
    hdrs = glob(["include/std_msgs/*.h"]),
)

rosmsg(
    name = "std_msgs",
    srcs = glob(["msg/*.msg"]),
)
