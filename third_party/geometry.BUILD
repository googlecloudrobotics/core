# Description:
#   Packages for common geometric calculations including the ROS transform
#   library, "tf".

load("@cloud_robotics//bazel/build_rules:rosmsg.bzl", "rosmsg")

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])  # BSD

rosmsg(
    name = "tf",
    srcs = glob([
        "tf/msg/*.msg",
        "tf/srv/*.srv",
    ]),
    deps = [
        "@com_github_ros_common_msgs//:actionlib_msgs",
        "@com_github_ros_common_msgs//:geometry_msgs",
    ],
)

cc_library(
    name = "tf_lib",
    srcs = [
        "tf/src/cache.cpp",
        "tf/src/tf.cpp",
        "tf/src/transform_broadcaster.cpp",
        "tf/src/transform_listener.cpp",
    ],
    hdrs = glob([
        "tf/include/tf/*.h",
        "tf/include/tf/LinearMath/*.h",
    ]),
    copts = ["-Iexternal/com_github_ros_ros_comm/utilities/xmlrpcpp/include/xmlrpcpp"],
    includes = ["tf/include"],
    deps = [
        ":tf_cpp",
        "@boost//:bind",
        "@boost//:function",
        "@boost//:shared_ptr",
        "@boost//:signals2",
        "@boost//:smart_ptr",
        "@boost//:thread",
        "@boost//:unordered",
        "@com_github_ros_angles//:angles",
        "@com_github_ros_common_msgs//:geometry_msgs_cpp",
        "@com_github_ros_common_msgs//:sensor_msgs_cpp",
        "@com_github_ros_geometry2//:tf2",
        "@com_github_ros_geometry2//:tf2_ros",
        "@com_github_ros_ros_comm//:message_filters",
        "@com_github_ros_ros_comm//:roscpp_lib",
        "@com_github_ros_ros_comm//:xmlrpcpp",
        "@com_github_ros_std_msgs//:std_msgs_cpp",
    ],
)
