# Description:
#   ROS communications-related packages, including core client libraries
#   (roscpp, rospy, roslisp) and graph introspection tools (rostopic, rosnode,
#   rosservice, rosparam).

load("@bazel_rules//:config.bzl", "cc_fix_config")
load("@cloud_robotics//bazel/build_rules:rosmsg.bzl", "rosmsg")

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])  # BSD

# TODO(rodrigoq): extract version number from package.xml
cc_fix_config(
    name = "ros_common_h",
    cmake = True,
    files = {"clients/roscpp/include/ros/common.h.in": "ros/common.h"},
    values = {
        "roscpp_VERSION_MAJOR": "1",
        "roscpp_VERSION_MINOR": "13",
        "roscpp_VERSION_PATCH": "2",
    },
)

cc_fix_config(
    name = "libros_config_h",
    cmake = True,
    files = {"clients/roscpp/src/libros/config.h.in": "libros_internal/config.h"},
    values = {
        "HAVE_IFADDRS_H": "1",
        "HAVE_TRUNC": "1",
    },
)

# (Ab-)Use cc_fix_config() to move the headers from the source directory to
# genfiles, and to remove the unwanted path prefix so they can be included with
# `includes = ["."]`. This is required so they can find ros/common.h.
roscpp_hdrs_src = glob([
    "clients/roscpp/include/*.h",
    "clients/roscpp/include/ros/*.h",
    "clients/roscpp/include/ros/transport/*.h",
])

roscpp_hdrs_out = [p.split("/", 3)[-1] for p in roscpp_hdrs_src]

cc_fix_config(
    name = "roscpp_hdrs",
    files = dict(zip(roscpp_hdrs_src, roscpp_hdrs_out)),
    values = {},
)

roscpp_hdrs = roscpp_hdrs_out + ["ros/common.h"]

rosmsg(
    name = "roscpp",
    srcs = glob([
        "clients/roscpp/msg/*.msg",
        "clients/roscpp/srv/*.srv",
    ]),
)

cc_library(
    name = "cpp_common",
    srcs = glob(["cpp_common/src/*.cpp"]),
    hdrs = glob(["cpp_common/include/ros/*.h"]),
    includes = ["cpp_common/include"],
    deps = [
        "@com_github_ros_console_bridge//:console_bridge",
    ],
)

cc_library(
    name = "rosbag_storage",
    srcs = glob([
        "tools/rosbag_storage/src/*.cpp",
    ]),
    hdrs = glob([
        "tools/rosbag_storage/include/rosbag/*.h",
    ]),
    copts = ["-Wno-unused-value"],
    includes = ["tools/rosbag_storage/include"],
    deps = [
        ":roslz4",
        "@boost//:foreach",
        "@boost//:format",
        "@boost//:function",
        "@boost//:iterator",
        "@boost//:shared_ptr",
        "@com_github_ros_console_bridge//:console_bridge",
        "@com_github_ros_roscpp_core//:cpp_common",
        "@com_github_ros_roscpp_core//:roscpp_serialization",
        "@com_github_ros_roscpp_core//:roscpp_traits",
        "@com_github_ros_roscpp_core//:rostime",
        "@org_bzip_bzip2//:bz2lib",
    ],
)

cc_library(
    name = "roscpp_lib",
    srcs = glob([
        "clients/roscpp/src/libros/*.cpp",
        "clients/roscpp/src/libros/transport/*.cpp",
    ]) + [
        "libros_internal/config.h",
    ],
    hdrs = roscpp_hdrs,
    copts = [
        "-I$(GENDIR)/external/com_github_ros_ros_comm/libros_internal",
        "-Wno-unused-function",
    ],
    includes = ["."],
    deps = [
        ":rosconsole",
        ":roscpp_cpp",
        ":xmlrpcpp",
        "@boost//:array",
        "@boost//:assert",
        "@boost//:bind",
        "@boost//:chrono",
        "@boost//:config",
        "@boost//:date_time",
        "@boost//:filesystem",
        "@boost//:function",
        "@boost//:lexical_cast",
        "@boost//:scope_exit",
        "@boost//:shared_array",
        "@boost//:shared_ptr",
        "@boost//:signals2",
        "@boost//:smart_ptr",
        "@boost//:thread",
        "@boost//:throw_exception",
        "@boost//:type_traits",
        "@boost//:utility",
        "@boost//:version",
        "@com_github_ros_ros_comm_msgs//:rosgraph_msgs_cpp",
        "@com_github_ros_ros_comm_msgs//:std_srvs_cpp",
        "@com_github_ros_roscpp_core//:cpp_common",
        "@com_github_ros_roscpp_core//:roscpp_serialization",
        "@com_github_ros_roscpp_core//:roscpp_traits",
        "@com_github_ros_roscpp_core//:rostime",
        "@com_github_ros_std_msgs//:std_msgs_cpp",
    ],
)

cc_library(
    name = "rosconsole",
    srcs = [
        "tools/rosconsole/src/rosconsole/impl/rosconsole_print.cpp",
        "tools/rosconsole/src/rosconsole/rosconsole.cpp",
        "tools/rosconsole/src/rosconsole/rosconsole_backend.cpp",
    ],
    hdrs = glob([
        "tools/rosconsole/include/ros/*.h",
        "tools/rosconsole/include/rosconsole/*.h",
    ]),
    includes = ["tools/rosconsole/include"],
    deps = [
        "@boost//:thread",
        "@com_github_ros_roscpp_core//:cpp_common",
        "@com_github_ros_roscpp_core//:rostime",
    ],
)

rosmsg(
    name = "topic_tools",
    srcs = glob([
        "tools/topic_tools/srv/*.srv",
    ]),
    deps = [
        "@com_github_ros_std_msgs//:std_msgs",
    ],
)

cc_library(
    name = "topic_tools",
    srcs = glob([
        "tools/topic_tools/src/*.cpp",
    ]),
    hdrs = glob([
        "tools/topic_tools/include/topic_tools/*.h",
    ]),
    includes = ["tools/topic_tools/include"],
    deps = [
        ":roscpp_lib",
        ":topic_tools_cpp",
        "@com_github_ros_roscpp_core//:cpp_common",
        "@com_github_ros_roscpp_core//:roscpp_serialization",
        "@com_github_ros_roscpp_core//:roscpp_traits",
        "@com_github_ros_roscpp_core//:rostime",
    ],
)

filegroup(
    name = "rosout_files",
    srcs = [
        "tools/rosout/package.xml",
    ],
)

cc_binary(
    name = "rosout",
    srcs = ["tools/rosout/rosout.cpp"],
    data = [":rosout_files"],
    linkopts = [
        "-lm",
    ],
    linkstatic = 1,
    deps = [
        ":roscpp_lib",
        "@com_github_ros_std_msgs//:std_msgs_cpp",
    ],
)

load("@ros_deps//:requirements.bzl", "requirement")

filegroup(
    name = "rosgraph_files",
    srcs = [
        "tools/rosgraph/conf/python_logging.conf",
        "tools/rosgraph/package.xml",
    ],
)

py_library(
    name = "rosgraph",
    srcs = glob([
        "tools/rosgraph/src/rosgraph/*.py",
        "tools/rosgraph/src/rosgraph/impl/*.py",
    ]),
    data = [":rosgraph_files"],
    imports = ["tools/rosgraph/src"],
    deps = [
        # TODO(rodrigoq): add dep on rospy if RosStreamHandler is needed
        "@com_github_ros_infrastructure_rospkg//:rospkg",
        requirement("defusedxml"),
        requirement("netifaces"),
    ],
)

py_library(
    name = "rosbag_lib",
    srcs = glob([
        "tools/rosbag/src/rosbag/*.py",
    ]),
    imports = ["tools/rosbag/src"],
    deps = [
        ":rospy",
        "@com_github_ros_genmsg//:genmsg",
        "@com_github_ros_genpy//:genpy",
        "@com_github_ros_infrastructure_rospkg//:rospkg",
        "@com_github_ros_ros//:roslib_py",
        requirement("PyYAML"),
    ],
)

py_library(
    name = "rosmsg_lib",
    srcs = glob([
        "tools/rosmsg/src/rosmsg/*.py",
    ]),
    imports = ["tools/rosmsg/src"],
    deps = [
        ":rosbag_lib",
        "@com_github_ros_catkin//:catkin",
        "@com_github_ros_genmsg//:genmsg",
        "@com_github_ros_genpy//:genpy",
        "@com_github_ros_infrastructure_rospkg//:rospkg",
        "@com_github_ros_ros//:roslib_py",
        requirement("PyYAML"),
    ],
)

filegroup(
    name = "rosnode_files",
    srcs = ["tools/rosnode/package.xml"],
)

py_library(
    name = "rosnode_lib",
    srcs = glob([
        "tools/rosnode/src/rosnode/*.py",
    ]),
    imports = ["tools/rosnode/src"],
    deps = [
        ":rosgraph",
        ":rostopic_lib",
    ],
)

py_library(
    name = "rosparam_lib",
    srcs = glob([
        "tools/rosparam/src/rosparam/*.py",
    ]),
    imports = ["tools/rosparam/src"],
    deps = [
        ":rosgraph",
        requirement("PyYAML"),
    ],
)

py_library(
    name = "rosservice_lib",
    srcs = glob([
        "tools/rosservice/src/rosservice/*.py",
    ]),
    imports = ["tools/rosservice/src"],
    deps = [
        ":rosgraph",
        ":rosmsg_lib",
        ":rospy",
        "@com_github_ros_genpy//:genpy",
        "@com_github_ros_ros//:roslib_py",
    ],
)

py_library(
    name = "rostest_lib",
    srcs = glob([
        "tools/rostest/src/rostest/*.py",
    ]),
    imports = ["tools/rostest/src"],
    deps = [
        ":rosgraph",
        ":roslaunch",
        ":rosmaster_lib",
        ":rospy",
        "@com_github_ros_infrastructure_rospkg//:rospkg",
        "@com_github_ros_ros//:roslib_py",
        "@com_github_ros_ros//:rosunit",
    ],
)

py_library(
    name = "rostopic_lib",
    srcs = glob([
        "tools/rostopic/src/rostopic/*.py",
    ]),
    imports = ["tools/rostopic/src"],
    deps = [
        ":rosgraph",
        ":rospy",
        "@com_github_ros_genpy//:genpy",
        "@com_github_ros_ros//:roslib_py",
        requirement("PyYAML"),
    ],
)

py_library(
    name = "rospy",
    srcs = glob([
        "clients/rospy/src/rospy/*.py",
        "clients/rospy/src/rospy/impl/*.py",
    ]),
    imports = ["clients/rospy/src"],
    deps = [
        ":roscpp_py",
        ":rosgraph",
        ":rosmaster_lib",
        ":rosparam_lib",
        "@com_github_ros_ros//:roslib_py",
        "@com_github_ros_ros_comm_msgs//:rosgraph_msgs_py",
        "@com_github_ros_std_msgs//:std_msgs_py",
        requirement("PyYAML"),
    ],
)

py_library(
    name = "rosmaster_lib",
    srcs = glob([
        "tools/rosmaster/src/rosmaster/*.py",
    ]),
    imports = ["tools/rosmaster/src"],
    deps = [
        ":rosgraph",
        requirement("defusedxml"),
    ],
)

# py_binary expects all files to end with .py, so we must rename the main
# script. Using `rosmaster_main.py` means that it doesn't get incorrectly
# picked up by `import rosmaster`.
genrule(
    name = "rename_rosmaster",
    srcs = ["tools/rosmaster/scripts/rosmaster"],
    outs = ["tools/rosmaster/scripts/rosmaster_main.py"],
    cmd = "cp $< $@",
)

py_binary(
    name = "rosmaster",
    srcs = [
        "tools/rosmaster/scripts/rosmaster_main.py",
    ],
    main = "tools/rosmaster/scripts/rosmaster_main.py",
    python_version = "PY2",
    deps = [
        ":rosmaster_lib",
    ],
)

filegroup(
    name = "roslaunch_files",
    srcs = [
        "tools/roslaunch/package.xml",
        "tools/roslaunch/resources/roscore.xml",
    ],
)

py_library(
    name = "roslaunch",
    srcs = glob([
        "tools/roslaunch/src/roslaunch/*.py",
    ]),
    data = [
        ":roslaunch_files",
        ":rosmaster",
        ":rosout",
        "@com_github_ros_infrastructure_rospkg//:rosversion",
    ],
    imports = ["tools/roslaunch/src"],
    deps = [
        ":rosgraph",
        ":rosmaster_lib",
        ":rosparam_lib",
        "@com_github_ros_catkin//:catkin",
        "@com_github_ros_infrastructure_catkin_pkg//:catkin_pkg",
        "@com_github_ros_infrastructure_rospkg//:rospkg",
        "@com_github_ros_ros//:rosclean",
        "@com_github_ros_ros//:roslib_py",
        "@com_github_ros_ros_comm_msgs//:rosgraph_msgs_py",
        requirement("PyYAML"),
    ],
)

cc_library(
    name = "message_filters",
    srcs = ["utilities/message_filters/src/connection.cpp"],
    hdrs = glob([
        "utilities/message_filters/include/message_filters/*.h",
        "utilities/message_filters/include/message_filters/sync_policies/*.h",
    ]),
    includes = ["utilities/message_filters/include"],
    deps = [
        ":rosconsole",
        ":roscpp_lib",
        "@boost//:signals2",
        "@com_github_ros_roscpp_core//:cpp_common",
        "@com_github_ros_roscpp_core//:roscpp_traits",
        "@com_github_ros_roscpp_core//:rostime",
    ],
)

cc_library(
    name = "roslz4",
    srcs = [
        "utilities/roslz4/src/lz4s.c",
        "utilities/roslz4/src/xxhash.c",
        "utilities/roslz4/src/xxhash.h",
    ],
    hdrs = [
        "utilities/roslz4/include/roslz4/lz4s.h",
    ],
    includes = ["utilities/roslz4/include"],
    deps = [
        "@com_github_lz4_lz4//:lz4",
    ],
)

cc_library(
    name = "xmlrpcpp",
    srcs = glob(["utilities/xmlrpcpp/src/*.cpp"]),
    hdrs = glob(["utilities/xmlrpcpp/include/xmlrpcpp/*.h"]),
    copts = ["-Wno-unused-variable"],
    includes = ["utilities/xmlrpcpp/include"],
    deps = [
        ":libb64",
        "@com_github_ros_roscpp_core//:cpp_common",
    ],
)

cc_library(
    name = "libb64",
    srcs = glob(["utilities/xmlrpcpp/libb64/src/*.c"]),
    hdrs = glob(["utilities/xmlrpcpp/libb64/include/b64/*.h"]),
    includes = ["utilities/xmlrpcpp/libb64/include"],
)
