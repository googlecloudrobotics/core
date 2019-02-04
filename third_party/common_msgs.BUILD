# Description:
#   Commonly used messages in ROS. Includes messages for diagnostics
#   (diagnostic_msgs), geometric primitives (geometry_msgs), robot navigation
#   (nav_msgs), and common sensors (sensor_msgs), such as laser range finders,
#   cameras, point clouds.

load("@cloud_robotics//bazel/build_rules:rosmsg.bzl", "rosmsg")

licenses(["notice"])  # BSD

package(
    default_visibility = ["//visibility:public"],
)

rosmsg(
    name = "actionlib_msgs",
    srcs = glob(["actionlib_msgs/msg/*.msg"]),
    data = ["actionlib_msgs/package.xml"],
    deps = ["@com_github_ros_std_msgs//:std_msgs"],
)

rosmsg(
    name = "diagnostic_msgs",
    srcs = glob([
        "diagnostic_msgs/msg/*.msg",
        "diagnostic_msgs/srv/*.srv",
    ]),
    data = ["diagnostic_msgs/package.xml"],
    deps = ["@com_github_ros_std_msgs//:std_msgs"],
)

rosmsg(
    name = "geometry_msgs",
    srcs = glob(["geometry_msgs/msg/*.msg"]),
    data = ["geometry_msgs/package.xml"],
    deps = ["@com_github_ros_std_msgs//:std_msgs"],
)

rosmsg(
    name = "nav_msgs",
    srcs = glob([
        "nav_msgs/msg/*.msg",
        "nav_msgs/srv/*.srv",
    ]),
    data = ["nav_msgs/package.xml"],
    deps = [
        ":geometry_msgs",
        "@com_github_ros_std_msgs//:std_msgs",
    ],
)

rosmsg(
    name = "sensor_msgs",
    srcs = glob([
        "sensor_msgs/msg/*.msg",
        "sensor_msgs/srv/*.srv",
    ]),
    hdrs = [
        "sensor_msgs/include/sensor_msgs/distortion_models.h",
        "sensor_msgs/include/sensor_msgs/fill_image.h",
        "sensor_msgs/include/sensor_msgs/image_encodings.h",
        "sensor_msgs/include/sensor_msgs/point_cloud_conversion.h",
        "sensor_msgs/include/sensor_msgs/point_field_conversion.h",
    ],
    data = ["sensor_msgs/package.xml"],
    deps = [
        ":geometry_msgs",
        "@com_github_ros_std_msgs//:std_msgs",
    ],
)

rosmsg(
    name = "shape_msgs",
    srcs = glob(["shape_msgs/msg/*.msg"]),
    data = ["shape_msgs/package.xml"],
    deps = [
        ":geometry_msgs",
        "@com_github_ros_std_msgs//:std_msgs",
    ],
)

rosmsg(
    name = "stereo_msgs",
    srcs = glob(["stereo_msgs/msg/*.msg"]),
    data = ["stereo_msgs/package.xml"],
    deps = [
        ":sensor_msgs",
        "@com_github_ros_std_msgs//:std_msgs",
    ],
)

rosmsg(
    name = "trajectory_msgs",
    srcs = glob(["trajectory_msgs/msg/*.msg"]),
    data = ["trajectory_msgs/package.xml"],
    deps = [
        ":geometry_msgs",
        "@com_github_ros_std_msgs//:std_msgs",
    ],
)

rosmsg(
    name = "visualization_msgs",
    srcs = glob(["visualization_msgs/msg/*.msg"]),
    data = ["visualization_msgs/package.xml"],
    deps = [
        ":geometry_msgs",
        "@com_github_ros_std_msgs//:std_msgs",
    ],
)

cc_library(
    name = "ps3joy_buttons",
    hdrs = ["sensor_msgs/ps3joy_buttons.h"],
)

py_binary(
    name = "genaction",
    srcs = ["actionlib_msgs/scripts/genaction.py"],
)
