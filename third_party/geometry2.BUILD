# Description:
#   A set of ROS packages for keeping track of coordinate transforms.

load("@cloud_robotics//bazel/build_rules:rosmsg.bzl", "rosmsg")

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])  # BSD

rosmsg(
    name = "tf2_msgs",
    srcs = glob([
        "tf2_msgs/action/*.action",
        "tf2_msgs/msg/*.msg",
        "tf2_msgs/srv/*.srv",
    ]),
    deps = [
        "@com_github_ros_common_msgs//:actionlib_msgs",
        "@com_github_ros_common_msgs//:geometry_msgs",
    ],
)

cc_library(
    name = "tf2",
    srcs = glob(["tf2/src/*.cpp"]),
    hdrs = glob([
        "tf2/include/tf2/*.h",
        "tf2/include/tf2/LinearMath/*.h",
        "tf2/include/tf2/impl/*.h",
    ]),
    copts = ["-Wno-sign-compare"],
    includes = ["tf2/include"],
    deps = [
        ":tf2_msgs_cpp",
        "@boost//:bind",
        "@boost//:foreach",
        "@boost//:function",
        "@boost//:lexical_cast",
        "@boost//:scoped_ptr",
        "@boost//:shared_ptr",
        "@boost//:signals2",
        "@boost//:thread",
        "@boost//:unordered",
        "@com_github_ros_common_msgs//:geometry_msgs_cpp",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "tf2_test_cache_unittest",
    srcs = ["tf2/test/cache_unittest.cpp"],
    deps = [
        ":tf2",
        "@com_github_ros_common_msgs//:geometry_msgs_cpp",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "tf2_test_simple_tf2_core",
    srcs = ["tf2/test/simple_tf2_core.cpp"],
    deps = [
        ":tf2",
        "@com_github_ros_ros_comm//:roscpp_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "tf2_test_speed_test",
    srcs = ["tf2/test/speed_test.cpp"],
    deps = [
        ":tf2",
        "@com_github_ros_console_bridge//:console_bridge",
        "@com_github_ros_ros_comm//:roscpp_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "tf2_test_static_cache_test",
    srcs = ["tf2/test/static_cache_test.cpp"],
    deps = [
        ":tf2",
        "@com_github_ros_common_msgs//:geometry_msgs_cpp",
        "@com_github_ros_ros_comm//:roscpp_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "tf2_eigen",
    hdrs = ["tf2_eigen/include/tf2_eigen/tf2_eigen.h"],
    includes = ["tf2_eigen/include"],
    deps = [
        ":tf2",
        "@com_github_ros_common_msgs//:geometry_msgs_cpp",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "tf2_ros",
    srcs = glob(
        ["tf2_ros/src/*.cpp"],
        exclude = [
            "tf2_ros/src/buffer_server_main.cpp",
            "tf2_ros/src/static_transform_broadcaster_program.cpp",
        ],
    ),
    hdrs = glob(["tf2_ros/include/tf2_ros/*.h"]),
    copts = ["-Wno-non-virtual-dtor"],
    includes = ["tf2_ros/include"],
    deps = [
        ":tf2",
        ":tf2_msgs_cpp",
        "@com_github_ros_actionlib//:actionlib",
        "@com_github_ros_common_msgs//:geometry_msgs_cpp",
        "@com_github_ros_ros_comm//:message_filters",
        "@com_github_ros_ros_comm//:rosconsole",
        "@com_github_ros_ros_comm//:roscpp_lib",
        "@com_github_ros_std_msgs//:std_msgs_cpp",
    ],
)
