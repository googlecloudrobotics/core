# Description:
#   std_msgs contains Point Cloud Library related messages.

load("@cloud_robotics//bazel/build_rules:rosmsg.bzl", "rosmsg")

licenses(["notice"])  # BSD

package(
    default_visibility = ["//visibility:public"],
)

rosmsg(
    name = "pcl_msgs",
    srcs = glob(["msg/*.msg"]),
    deps = [
        "@com_github_ros_common_msgs//:sensor_msgs",
        "@com_github_ros_std_msgs//:std_msgs",
    ],
)
