# Description:
#   The Point Cloud Library (PCL) is a standalone, large scale, open project
#   for 2D/3D image and point cloud processing.

load("@bazel_rules//:config.bzl", "cc_fix_config")

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])  # BSD

# TODO(rodrigoq): extract version number etc
cc_fix_config(
    name = "common_pcl_config",
    cmake = True,
    files = {"pcl_config.h.in": "common/include/pcl/pcl_config.h"},
    values = {
        "CMAKE_BUILD_TYPE": "RelWithDebInfo",
        "PCL_MAJOR_VERSION": "1",
        "PCL_MINOR_VERSION": "8",
        "PCL_REVISION_VERSION": "1",
        "PCL_DEV_VERSION": "1",
        "PCL_VERSION": "1.8.1-dev",
        "HAVE_OPENNI": "1",
        "HAVE_QHULL": "1",
        "HAVE_POSIX_MEMALIGN": "1",
        "HAVE_SSE4_2_EXTENSIONS": "1",
        "HAVE_SSE4_1_EXTENSIONS": "1",
        "HAVE_SSSE3_EXTENSIONS": "1",
        "HAVE_SSE3_EXTENSIONS": "1",
        "HAVE_SSE2_EXTENSIONS": "1",
        "HAVE_SSE_EXTENSIONS": "1",
        "HAVE_PNG": "1",
        "VERBOSITY_LEVEL_INFO": "1",
        "VTK_RENDERING_BACKEND_OPENGL_VERSION": "1",
    },
    visibility = ["//visibility:private"],
)

cc_library(
    name = "common",
    srcs = glob([
        "common/src/*.cpp",
        "common/src/fft/*.cpp",
    ]),
    hdrs = glob(
        [
            "common/include/pcl/*.h",
            "common/include/pcl/common/*.h",
            "common/include/pcl/common/fft/*.h",
            "common/include/pcl/console/*.h",
            "common/include/pcl/range_image/*.h",
            "common/include/pcl/ros/*.h",
        ],
        exclude = [
            "common/include/pcl/pcl_tests.h",
        ],
    ) + [
        "common/include/pcl/pcl_config.h",
    ],
    copts = [
        "-Wno-unknown-pragmas",
        "-Wno-error=unknown-pragmas",
        "-Wno-comment",
    ],
    includes = ["common/include"],
    textual_hdrs = glob([
        "common/include/pcl/common/impl/*.hpp",
        "common/include/pcl/impl/*.hpp",
        "common/include/pcl/range_image/impl/*.hpp",
    ]),
    deps = [
        "@boost//:algorithm",
        "@boost//:config",
        "@boost//:current_function",
        "@boost//:date_time",
        "@boost//:detail",
        "@boost//:foreach",
        "@boost//:function",
        "@boost//:fusion",
        "@boost//:mpl",
        "@boost//:preprocessor",
        "@boost//:random",
        "@boost//:shared_ptr",
        "@boost//:signals2",
        "@boost//:smart_ptr",
        "@boost//:thread",
        "@boost//:type_traits",
        "@boost//:utility",
        "@boost//:version",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

cc_library(
    name = "io",
    srcs = [
        "io/src/lzf.cpp",
        "io/src/pcd_io.cpp",
    ],
    hdrs = [
        "io/include/pcl/io/boost.h",
        "io/include/pcl/io/file_io.h",
        "io/include/pcl/io/impl/pcd_io.hpp",
        "io/include/pcl/io/lzf.h",
        "io/include/pcl/io/pcd_io.h",
    ],
    copts = [
        "$(STACK_FRAME_UNLIMITED)",
        "-Wno-implicit-fallthrough",
    ],
    includes = ["io/include"],
    deps = [
        ":common",
        "@boost//:algorithm",
        "@boost//:array",
        "@boost//:asio",
        "@boost//:bind",
        "@boost//:chrono",
        "@boost//:circular_buffer",
        "@boost//:config",
        "@boost//:date_time",
        "@boost//:detail",
        "@boost//:filesystem",
        "@boost//:foreach",
        "@boost//:format",
        "@boost//:interprocess",
        "@boost//:iostreams",
        "@boost//:lexical_cast",
        "@boost//:math",
        "@boost//:mpl",
        "@boost//:numeric",
        "@boost//:shared_array",
        "@boost//:shared_ptr",
        "@boost//:signals2",
        "@boost//:smart_ptr",
        "@boost//:thread",
        "@boost//:tokenizer",
        "@boost//:tuple",
        "@boost//:utility",
        "@boost//:version",
    ],
)
