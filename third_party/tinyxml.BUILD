# Description:
#   A simple, small, C++ XML parser

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])  # BSD

cc_library(
    name = "tinyxml",
    srcs = [
        "tinystr.cpp",
        "tinyxml.cpp",
        "tinyxmlerror.cpp",
        "tinyxmlparser.cpp",
    ],
    hdrs = [
        "tinystr.h",
        "tinyxml.h",
    ],
    defines = ["TIXML_USE_STL"],
    includes = ["."],
)
