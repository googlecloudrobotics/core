# Description:
#   Base dependencies and support libraries for ROS.

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])  # BSD

load("@ros_deps//:requirements.bzl", "requirement")

cc_library(
    name = "roslib",
    srcs = ["core/roslib/src/package.cpp"],
    hdrs = ["core/roslib/include/ros/package.h"],
    includes = ["core/roslib/include"],
    deps = [
        "@boost//:algorithm",
        "@boost//:thread",
        "@com_github_ros_rospack//:rospack",
    ],
)

py_library(
    name = "roslib_py",
    srcs = glob([
        "core/roslib/src/ros/*.py",
        "core/roslib/src/roslib/*.py",
    ]),
    imports = ["core/roslib/src"],
    deps = [
        "@com_github_ros_catkin//:catkin",
        "@com_github_ros_infrastructure_rospkg//:rospkg",
        requirement("netifaces"),
    ],
)

py_library(
    name = "rosclean",
    srcs = glob([
        "tools/rosclean/src/rosclean/*.py",
    ]),
    imports = ["tools/rosclean/src"],
    deps = [
        "@com_github_ros_infrastructure_rospkg//:rospkg",
    ],
)

py_library(
    name = "rosunit",
    srcs = glob([
        "tools/rosunit/src/rosunit/*.py",
    ]),
    imports = ["tools/rosunit/src"],
    deps = [
        "@com_github_ros_infrastructure_rospkg//:rospkg",
    ],
)
