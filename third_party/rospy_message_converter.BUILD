# Description:
#   Converts between Python dictionaries and JSON to ROS messages.

licenses(["notice"])  # BSD

package(default_visibility = ["//visibility:public"])

py_library(
    name = "rospy_message_converter",
    srcs = glob(["src/rospy_message_converter/*.py"]),
    imports = ["src"],
    deps = [
        "@com_github_ros_ros//:roslib_py",
        "@com_github_ros_ros_comm//:rospy",
    ],
)
