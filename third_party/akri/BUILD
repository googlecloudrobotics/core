
load("//googlex/giza/kubernetes:helm_chart.bzl", "helm_chart")

package(default_visibility = ["//googlex/giza:__subpackages__"])

helm_chart(
    name = "akri",
    exclusive = False,
    extra_templates = glob(["templates/*.yaml",]),
)

exports_files(
    srcs= ["values.yaml"]
)
