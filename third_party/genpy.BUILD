# Description:
#   ROS Python message definition and serialization generators.

licenses(["notice"])  # BSD

package(
    default_visibility = ["//visibility:public"],
)

load("@ros_deps//:requirements.bzl", "requirement")

py_library(
    name = "genpy",
    srcs = glob(["src/genpy/*.py"]),
    imports = ["src"],
    deps = [
        "@com_github_ros_genmsg//:genmsg",
        requirement("PyYAML"),
    ],
)

py_binary(
    name = "genmsg_py",
    srcs = ["scripts/genmsg_py.py"],
    python_version = "PY2",
    deps = [
        ":genpy",
    ],
)

py_binary(
    name = "gensrv_py",
    srcs = ["scripts/gensrv_py.py"],
    python_version = "PY2",
    deps = [
        ":genpy",
    ],
)
