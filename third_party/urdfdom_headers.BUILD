# Description:
#   Headers for URDF parsers.

package(
    default_visibility = ["//visibility:public"],
)

licenses(["notice"])  # BSD

cc_library(
    name = "urdf_exception",
    hdrs = glob(["urdf_exception/include/urdf_exception/*.h"]),
    includes = ["urdf_exception/include"],
)

cc_library(
    name = "urdf_model",
    hdrs = glob(["urdf_model/include/urdf_model/*.h"]),
    includes = ["urdf_model/include"],
    deps = [
        ":urdf_exception",
    ],
)

cc_library(
    name = "urdf_model_state",
    hdrs = glob(["urdf_model_state/include/urdf_model_state/*.h"]),
    includes = ["urdf_model_state/include"],
    deps = [
        ":urdf_model",
    ],
)

cc_library(
    name = "urdf_sensor",
    hdrs = glob(["urdf_sensor/include/urdf_sensor/*.h"]),
    includes = ["urdf_sensor/include"],
    deps = [
        ":urdf_model",
    ],
)

cc_library(
    name = "urdf_world",
    hdrs = glob(["urdf_world/include/urdf_world/*.h"]),
    includes = ["urdf_world/include"],
    deps = [
        "@com_github_icebreaker_tinyxml//:tinyxml",
    ],
)
