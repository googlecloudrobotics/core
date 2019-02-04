licenses(["notice"])  # Apache v2.0

py_library(
    name = "googleapiclient",
    srcs = glob(["googleapiclient/**"]),
    srcs_version = "PY2AND3",
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_google_oauth2client//:oauth2client",
        "@org_python_pypi_six//:six",
        "@org_python_pypi_uritemplate//:uritemplate",
    ],
)
