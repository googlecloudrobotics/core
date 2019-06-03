# Description:
#   Packaging system for ROS.

licenses(["notice"])  # BSD

package(default_visibility = ["//visibility:public"])

py_library(
    name = "rospkg",
    srcs = glob(["src/rospkg/*.py"]),
    imports = ["src"],
)

# py_binary expects all files to end with .py, so we must rename the main
# script.
genrule(
    name = "rename_rosversion",
    srcs = ["scripts/rosversion"],
    outs = ["scripts/rosversion.py"],
    cmd = "cp $< $@",
)

py_binary(
    name = "rosversion",
    srcs = ["scripts/rosversion.py"],
    main = "scripts/rosversion.py",
    python_version = "PY2",
    deps = [":rospkg"],
)
