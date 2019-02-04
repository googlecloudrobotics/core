# Description:
#   ROS messages for ros_comm. Contains common services and messages for the
#   ROS Computation Graph.

load("@cloud_robotics//bazel/build_rules:rosmsg.bzl", "rosmsg")

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])  # BSD

rosmsg(
    name = "rosgraph_msgs",
    srcs = glob(["rosgraph_msgs/msg/*.msg"]),
    deps = ["@com_github_ros_std_msgs//:std_msgs"],
)

rosmsg(
    name = "std_srvs",
    srcs = glob(["std_srvs/srv/*.srv"]),
)
