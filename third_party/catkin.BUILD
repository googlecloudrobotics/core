# Description:
#   A CMake-based build system that is used to build all packages in ROS.

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])  # BSD

py_library(
    name = "catkin",
    srcs = glob([
        "python/catkin/*.py",
    ]),
    imports = ["python"],
    deps = [
        "@com_github_ros_infrastructure_catkin_pkg//:catkin_pkg",
    ],
)

# py_binary expects all files to end with .py, so we must rename the main
# script.
genrule(
    name = "rename_catkin_find",
    srcs = ["bin/catkin_find"],
    outs = ["bin/catkin_find.py"],
    cmd = "cp $< $@",
)

py_binary(
    name = "catkin_find",
    srcs = ["bin/catkin_find.py"],
    main = "bin/catkin_find.py",
    python_version = "PY2",
    deps = [":catkin"],
)
