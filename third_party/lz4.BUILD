# Description:
#   LZ4 is a lossless compression algorithm with very fast decompression.

licenses(["notice"])  # New BSD

cc_library(
    name = "lz4",
    srcs = glob(["lib/*.c"]),
    hdrs = glob([
        "lib/*.h",
        "lib/lz4.c",
    ]),
    copts = ["$(STACK_FRAME_UNLIMITED)"],
    defines = [
        # 'XXH_STATIC_LINKING_ONLY' makes xxhash exposing "XXH32_state_t" struct
        # definitions, which is required by LZ4.
        "XXH_STATIC_LINKING_ONLY=",
    ],
    includes = ["lib"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_cyan4973_xxhash//:xxhash",
    ],
)
