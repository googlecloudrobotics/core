# Description:
#   ROS messages for the navigation stack.

load("@cloud_robotics//bazel/build_rules:rosmsg.bzl", "rosmsg")

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])  # BSD

rosmsg(
    name = "map_msgs",
    srcs = glob([
        "map_msgs/msg/*.msg",
        "map_msgs/srv/*.srv",
    ]),
    data = ["map_msgs/package.xml"],
    deps = [
        "@com_github_ros_common_msgs//:nav_msgs",
        "@com_github_ros_common_msgs//:sensor_msgs",
        "@com_github_ros_std_msgs//:std_msgs",
    ],
)

rosmsg(
    name = "move_base_msgs",
    srcs = glob([
        "move_base_msgs/action/*.action",
    ]),
    data = ["move_base_msgs/package.xml"],
    deps = [
        "@com_github_ros_common_msgs//:actionlib_msgs",
        "@com_github_ros_common_msgs//:geometry_msgs",
        "@com_github_ros_std_msgs//:std_msgs",
    ],
)
