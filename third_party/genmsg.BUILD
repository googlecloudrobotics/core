# Description:
#   Standalone Python library for generating ROS message and service data
#   structures for various languages.

licenses(["notice"])  # BSD

py_library(
    name = "genmsg",
    srcs = glob(["src/genmsg/*.py"]),
    imports = ["src"],
    visibility = ["//visibility:public"],
    deps = ["@empy_repo//:empy"],
)

# TODO(rodrigoq): how can we run the tests? I think we need a custom
# py_nosetest as genmsg uses nose tests, which py_test doesn't run.
