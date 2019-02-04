# Description:
#   Robot Operating System core cpp libraries.

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])  # BSD

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
    name = "rostime",
    srcs = glob(["rostime/src/**/*.cpp"]),
    hdrs = glob(["rostime/include/**/*.h"]),
    copts = [
        "-Wno-narrowing",
    ],
    includes = ["rostime/include"],
    deps = [
        ":cpp_common",
        "@boost//:date_time",
        "@boost//:math",
        "@boost//:thread",
    ],
)

cc_library(
    name = "roscpp_traits",
    hdrs = glob(["roscpp_traits/include/**/*.h"]),
    includes = ["roscpp_traits/include"],
    deps = [
        ":cpp_common",
        ":rostime",
    ],
)

cc_library(
    name = "roscpp_serialization",
    srcs = glob(["roscpp_serialization/src/**/*.cpp"]),
    hdrs = glob(["roscpp_serialization/include/**/*.h"]),
    includes = ["roscpp_serialization/include"],
    deps = [
        ":cpp_common",
        ":roscpp_traits",
        ":rostime",
        "@boost//:array",
        "@boost//:call_traits",
        "@boost//:math",
    ],
)
