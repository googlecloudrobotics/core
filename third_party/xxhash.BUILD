# Description:
#   xxHash is an extremely fast non-cryptographic hash algorithm.
#   http://www.xxhash.com/

licenses(["notice"])  # BSD-2-Clause

cc_library(
    name = "xxhash",
    srcs = ["xxhash.c"],
    hdrs = ["xxhash.h"],
    # Evades symbol naming collisions since other version of xxhash.cc might
    # be vendored.
    defines = ["XXH_NAMESPACE=xxhash_"],
    includes = ["."],
    visibility = ["//visibility:public"],
)
