# Description:
#   A command-line tool for retrieving information about ROS packages available
#   on the filesystem.

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])  # BSD

cc_library(
    name = "rospack",
    srcs = [
        "src/rospack.cpp",
        "src/rospack_backcompat.cpp",
        "src/rospack_cmdline.cpp",
        "src/rospack_cmdline.h",
        "src/utils.cpp",
    ],
    hdrs = [
        "include/rospack/macros.h",
        "include/rospack/rospack.h",
        "include/rospack/rospack_backcompat.h",
        "src/utils.h",
    ],
    copts = [
        "-Wno-unused-variable",
    ],
    includes = ["include"],
    deps = [
        "@boost//:algorithm",
        "@boost//:filesystem",
        "@boost//:functional",
        "@boost//:program_options",
        "@boost//:unordered",
        "@com_github_leethomason_tinyxml2//:tinyxml2",
        "@python_linux_x86_64//:python27-lib",
    ],
)
