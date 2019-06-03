# Description:
#   ROS C++ message definition and serialization generators.

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])  # BSD

filegroup(
    name = "msg_template",
    srcs = ["scripts/msg.h.template"],
)

filegroup(
    name = "srv_template",
    srcs = ["scripts/srv.h.template"],
)

py_binary(
    name = "gencpp",
    srcs = ["scripts/gen_cpp.py"],
    main = "scripts/gen_cpp.py",
    python_version = "PY2",
    deps = [
        ":gencpp_lib",
        "@com_github_ros_genmsg//:genmsg",
    ],
)

py_library(
    name = "gencpp_lib",
    srcs = ["src/gencpp/__init__.py"],
    imports = ["src"],
    visibility = ["//visibility:private"],
    deps = [
        "@com_github_ros_genmsg//:genmsg",
    ],
)
