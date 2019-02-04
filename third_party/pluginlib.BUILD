# Description:
#   Library for loading/unloading plugins in ROS packages during runtime.

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])  # BSD

cc_library(
    name = "pluginlib",
    hdrs = glob(["include/pluginlib/*.h"]),
    includes = ["include"],
    deps = [
        "@boost//:algorithm",
        "@boost//:filesystem",
        "@com_github_icebreaker_tinyxml//:tinyxml",
        "@com_github_ros_class_loader//:class_loader",
        "@com_github_ros_ros_comm//:roscpp_lib",
    ],
)

## Plugin test
#
# These rules adapt the test in the upstream repository to run in Bazel. This
# is useful if you're working on the way the Bazel build makes plugins work.

# The following genrules put test's `data` dependencies in a directory called
# pluginlib, so that the ROS runtime can find them.
genrule(
    name = "package_xml",
    srcs = ["package.xml"],
    outs = ["pluginlib/package.xml"],
    cmd = "cp $< $@",
    output_to_bindir = True,
)

genrule(
    name = "test_plugins_xml",
    srcs = ["test/test_plugins.xml"],
    outs = ["pluginlib/test/test_plugins.xml"],
    cmd = "cp $< $@",
    output_to_bindir = True,
)

genrule(
    name = "test_plugins_broken_xml",
    srcs = ["test/test_plugins_broken.xml"],
    outs = ["pluginlib/test/test_plugins_broken.xml"],
    cmd = "cp $< $@",
    output_to_bindir = True,
)

# This empty .so will be loaded by pluginlib instead of the plugins. The actual
# plugins are statically linked.
cc_binary(
    name = "pluginlib/libtest_plugins.so",
    linkshared = 1,
)

# The plugin sources must be included in the `srcs` of the cc_binary or cc_test
# that uses the plugins to statically link them in. This is because the plugins
# rely on the execution of non-local static constructors, which would be
# removed by the linker if they were in a cc_library.
filegroup(
    name = "test_plugins",
    srcs = [
        "test/test_plugins.cpp",
        "test/test_plugins.h",
    ],
)

# This test must be run with:
#   --test_env ROS_PACKAGE_PATH=$PWD/bazel-bin/external/com_github_ros_pluginlib
# so that it can find the data files.
cc_test(
    name = "plugin_test",
    size = "small",
    srcs = [
        "test/test_base.h",
        "test/utest.cpp",
        ":test_plugins",
    ],
    data = [
        "pluginlib/libtest_plugins.so",
        "pluginlib/package.xml",
        "pluginlib/test/test_plugins.xml",
        "pluginlib/test/test_plugins_broken.xml",
    ],
    deps = [
        ":pluginlib",
        "@boost//:shared_ptr",
        "@com_google_googletest//:gtest",
    ],
)
