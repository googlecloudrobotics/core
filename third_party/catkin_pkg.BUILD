# Description:
#   Standalone Python library for the catkin build system.

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])  # BSD

py_library(
    name = "catkin_pkg",
    srcs = glob([
        "src/catkin_pkg/*.py",
        "src/catkin_pkg/cli/*.py",
    ]),
    imports = ["src"],
)
