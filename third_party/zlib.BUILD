# Description:
#   zlib is a general purpose data compression library.

licenses(["notice"])  # BSD/MIT-like license (for zlib)

config_setting(
    name = "linux_x86_64",
    values = {"cpu": "k8"},
    visibility = ["//visibility:public"],
)

cc_library(
    name = "zlib",
    srcs = [
        "adler32.c",
        "compress.c",
        "crc32.c",
        "crc32.h",
        "deflate.c",
        "deflate.h",
        "gzclose.c",
        "gzguts.h",
        "gzlib.c",
        "gzread.c",
        "gzwrite.c",
        "infback.c",
        "inffast.c",
        "inffast.h",
        "inffixed.h",
        "inflate.c",
        "inflate.h",
        "inftrees.c",
        "inftrees.h",
        "trees.c",
        "trees.h",
        "uncompr.c",
        "zconf.h",
        "zutil.c",
        "zutil.h",
    ] + select({
        "//:linux_x86_64": [
            "contrib/amd64/amd64-match.S",
        ],
        "//conditions:default": [
        ],
    }),
    hdrs = [
        "zlib.h",
    ],
    copts = [
        "-DUNALIGNED_OK",
        "-DEXPAND_INSERT_STRING",
        "-Wno-implicit-function-declaration",
    ],
    includes = ["."],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "zlib_test",
    size = "small",
    srcs = ["test/example.c"],
    deps = [
        ":zlib",
    ],
)
