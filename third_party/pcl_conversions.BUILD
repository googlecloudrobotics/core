# Description:
#   This package provides conversions from PCL data types and ROS message types.

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])  # BSD

cc_library(
    name = "pcl_conversions",
    hdrs = ["include/pcl_conversions/pcl_conversions.h"],
    includes = ["include"],
    deps = [
        "@com_github_pointcloudlibrary_pcl//:common",
        "@com_github_pointcloudlibrary_pcl//:io",
        "@com_github_ros_common_msgs//:sensor_msgs_cpp",
        "@com_github_ros_perception_pcl_msgs//:pcl_msgs_cpp",
        "@com_github_ros_ros_comm//:roscpp_lib",
        "@com_github_ros_std_msgs//:std_msgs_cpp",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_test(
    name = "pcl_conversions_test",
    size = "small",
    srcs = ["test/test_pcl_conversions.cpp"],
    copts = ["-Wno-implicit-fallthrough"],
    deps = [
        ":pcl_conversions",
        "@com_google_googletest//:gtest",
    ],
)
