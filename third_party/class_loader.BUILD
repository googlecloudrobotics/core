# Description:
#   ROS-independent library for dynamic class (i.e. plugin) introspection and
#   loading from runtime libraries.

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])  # BSD

cc_library(
    name = "class_loader",
    srcs = glob(["src/*.cpp"]),
    hdrs = glob(["include/class_loader/*.h"]),
    copts = ["-Wno-delete-non-virtual-dtor"],
    includes = ["include"],
    deps = [
        "@boost//:bind",
        "@boost//:thread",
        "@com_github_ros_console_bridge//:console_bridge",
        "@com_github_ros_ros//:roslib",
        "@org_pocoproject_poco//:foundation",
    ],
)
