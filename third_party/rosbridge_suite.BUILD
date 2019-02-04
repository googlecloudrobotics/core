# Description:
#   Server Implementations of the rosbridge v2 Protocol
#   http://robotwebtools.org/
#
load("@cloud_robotics//bazel/build_rules:copy.bzl", "copy_files")
load("@cloud_robotics//bazel/build_rules:rosmsg.bzl", "rosmsg")
load("@ros_deps//:requirements.bzl", "requirement")

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])  # BSD

py_library(
    name = "internal_backports",
    srcs = glob([
        "rosbridge_server/src/backports/*.py",
        "rosbridge_server/src/backports/ssl_match_hostname/*.py",
    ]),
    imports = ["rosbridge_server/src"],
)

py_library(
    name = "internal_tornado",
    srcs = glob([
        "rosbridge_server/src/tornado/*.py",
        "rosbridge_server/src/tornado/platform/*.py",
    ]),
    imports = ["rosbridge_server/src"],
    deps = [
        ":internal_backports",
        requirement("Twisted"),
    ],
)

py_library(
    name = "rosbridge_library",
    srcs = glob([
        "rosbridge_library/src/rosbridge_library/*.py",
        "rosbridge_library/src/rosbridge_library/capabilities/*.py",
        "rosbridge_library/src/rosbridge_library/internal/*.py",
        "rosbridge_library/src/rosbridge_library/util/*.py",
    ]),
    imports = ["rosbridge_library/src"],
    deps = [
        "@com_github_ros_ros//:roslib_py",
        "@com_github_ros_ros_comm//:rospy",
        "@com_github_ros_ros_comm//:rosservice_lib",
        "@com_github_ros_ros_comm//:rostopic_lib",
        requirement("pymongo"),
        requirement("Pillow"),
    ],
)

filegroup(
    name = "rosbridge_server_files",
    srcs = [
        "rosbridge_server/launch/rosbridge_websocket.launch",
        "rosbridge_server/package.xml",
    ],
)

py_library(
    name = "rosbridge_server",
    srcs = glob([
        "rosbridge_server/src/rosbridge_server/*.py",
    ]),
    imports = ["rosbridge_server/src"],
    deps = [
        ":internal_tornado",
        ":rosbridge_library",
        "@com_github_gt_rail_rosauth//:rosauth_msgs_py",
        "@com_github_ros_ros_comm//:rospy",
        requirement("Twisted"),
    ],
)

rosmsg(
    name = "rosapi_msgs",
    srcs = glob([
        "rosapi/msg/*.msg",
        "rosapi/srv/*.srv",
    ]),
    data = ["rosapi/package.xml"],
    rospkg = "rosapi",
)

# This file turns `rosapi` into a Python namespace, preventing conflict with
# the `rosapi` directory created by the rosmsg() macro.
genrule(
    name = "namespaced_init_py",
    outs = ["__namespaced__/rosapi/__init__.py"],
    cmd = """cat > $(@) << 'EOF'
try:
    import pkg_resources
    pkg_resources.declare_namespace(__name__)
except ImportError:
    import pkgutil
    __path__ = pkgutil.extend_path(__path__, __name__)
EOF
""",
)

copy_files(
    name = "rosapi_srcs",
    srcs = [
        "rosapi/src/rosapi/glob_helper.py",
        "rosapi/src/rosapi/objectutils.py",
        "rosapi/src/rosapi/params.py",
        "rosapi/src/rosapi/proxy.py",
    ],
    outdir = "__namespaced__/rosapi",
)

py_library(
    name = "rosapi",
    srcs = [
        "__namespaced__/rosapi/__init__.py",
        ":rosapi_srcs",
    ],
    data = [
        # These targets bring in the package.xml files, which are required
        # because rosapi uses `from ros import rosnode` to import from a
        # package located at runtime.
        "@com_github_ros_ros_comm//:rosgraph_files",
        "@com_github_ros_ros_comm//:rosnode_files",
    ],
    imports = ["__namespaced__"],
    deps = [
        ":rosapi_msgs_py",
        ":rosbridge_library",
        "@com_github_gt_rail_rosauth//:rosauth_msgs_py",
        "@com_github_ros_ros//:roslib_py",
        "@com_github_ros_ros_comm//:rosgraph",
        "@com_github_ros_ros_comm//:rosnode_lib",
        "@com_github_ros_ros_comm//:rospy",
        "@com_github_ros_ros_comm//:rosservice_lib",
        "@com_github_ros_ros_comm//:rostopic_lib",
        requirement("Twisted"),
    ],
)

# py_binary expects all files to end with .py. We also move it out of rosapi/,
# otherwise Bazel will create an empty rosapi/__init__.py and interfere with our
# attempts to make `rosapi` a namespace. https://github.com/bazelbuild/bazel/issues/3998
#
# TODO(rodrigoq): if #3998 is resolved, or if we can make rules_docker work with
# --noexperimental_python_import_all_repositories, see if we can avoid changing
# the path.
genrule(
    name = "rename_rosapi_node",
    srcs = ["rosapi/scripts/rosapi_node"],
    outs = ["__scripts__/rosapi_node.py"],
    cmd = "cp $< $@",
)

py_binary(
    name = "rosapi_node",
    srcs = ["__scripts__/rosapi_node.py"],
    data = ["rosapi/package.xml"],
    deps = [
        ":rosapi",
        ":rosapi_msgs_py",
        "@com_github_ros_ros_comm//:rospy",
    ],
)

py_binary(
    name = "rosbridge_websocket",
    srcs = ["rosbridge_server/scripts/rosbridge_websocket.py"],
    deps = [
        ":internal_tornado",
        ":rosbridge_library",
        ":rosbridge_server",
    ],
)
