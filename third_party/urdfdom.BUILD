# Description:
#   The URDF (U-Robot Description Format) library provides core data structures
#   and a simple XML parsers for populating the class data structures from an
#   URDF file.

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])  # BSD

cc_library(
    name = "urdf_parser",
    srcs = glob(
        ["urdf_parser/src/*.cpp"],
        exclude = [
            "urdf_parser/src/check_urdf.cpp",
            "urdf_parser/src/urdf_to_graphiz.cpp",
        ],
    ),
    hdrs = glob(["urdf_parser/include/urdf_parser/*.h"]),
    includes = ["urdf_parser/include"],
    deps = [
        "@com_github_icebreaker_tinyxml//:tinyxml",
        "@com_github_ros_console_bridge//:console_bridge",
        "@com_github_ros_urdfdom_headers//:urdf_model",
        "@com_github_ros_urdfdom_headers//:urdf_model_state",
        "@com_github_ros_urdfdom_headers//:urdf_sensor",
        "@com_github_ros_urdfdom_headers//:urdf_world",
    ],
)
