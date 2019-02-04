# Description:
#   Server Side tools for Authorization and Authentication of ROS Clients
#
load("@cloud_robotics//bazel/build_rules:rosmsg.bzl", "rosmsg")

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])  # BSD

rosmsg(
    name = "rosauth_msgs",
    srcs = ["srv/Authentication.srv"],
    rospkg = "rosauth",
)
