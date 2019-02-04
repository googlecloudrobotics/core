# Description:
#   Lua language interpreter.

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])  # MIT

cc_library(
    name = "lua_includes",
    hdrs = [
        "src/lauxlib.h",
        "src/lua.h",
        "src/lua.hpp",
        "src/luaconf.h",
        "src/lualib.h",
    ],
    includes = ["src"],
)

cc_library(
    name = "lua",
    srcs = [
        # Core language
        "src/lapi.c",
        "src/lapi.h",
        "src/lcode.c",
        "src/lcode.h",
        "src/lctype.c",
        "src/lctype.h",
        "src/ldebug.c",
        "src/ldebug.h",
        "src/ldo.c",
        "src/ldo.h",
        "src/ldump.c",
        "src/lfunc.c",
        "src/lfunc.h",
        "src/lgc.c",
        "src/lgc.h",
        "src/llex.c",
        "src/llex.h",
        "src/llimits.h",
        "src/lmem.c",
        "src/lmem.h",
        "src/lobject.c",
        "src/lobject.h",
        "src/lopcodes.c",
        "src/lopcodes.h",
        "src/lparser.c",
        "src/lparser.h",
        "src/lstate.c",
        "src/lstate.h",
        "src/lstring.c",
        "src/lstring.h",
        "src/ltable.c",
        "src/ltable.h",
        "src/ltm.c",
        "src/ltm.h",
        "src/lundump.c",
        "src/lundump.h",
        "src/lvm.c",
        "src/lvm.h",
        "src/lzio.c",

        # Standard libraries
        "src/lauxlib.c",
        "src/lbaselib.c",
        "src/lbitlib.c",
        "src/lcorolib.c",
        "src/ldblib.c",
        "src/linit.c",
        "src/liolib.c",
        "src/lmathlib.c",
        "src/loadlib.c",
        "src/loslib.c",
        "src/lstrlib.c",
        "src/ltablib.c",
        "src/lzio.h",
    ],
    hdrs = [
        "src/lauxlib.h",
        "src/lua.h",
        "src/lua.hpp",
        "src/luaconf.h",
        "src/lualib.h",
    ],
    copts = ["-w"],
    defines = ["LUA_USE_LINUX"],
    includes = ["src"],
    linkopts = [
        "-lm",
        "-ldl",
    ],
)
