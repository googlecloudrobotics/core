# Description:
#   A ROS-independent package for logging that seamlessly pipes into
#   rosconsole/rosout for ROS-dependent packages.

licenses(["notice"])  # BSD

cc_library(
    name = "console_bridge",
    srcs = ["src/console.cpp"],
    hdrs = glob(["include/console_bridge/*.h"]),
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = ["@boost//:regex"],
)
