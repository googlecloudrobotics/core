def helm_template(name, release_name, chart, values, namespace = None):
    """Locally expand a helm chart.

    Args:
      chart: build label, referencing the chart to expand.
      values: label. File with expand-time values.
    """
    native.genrule(
        name = name,
        srcs = [chart, values],
        outs = [name + ".yaml"],
        cmd = "$(location @kubernetes_helm//:helm) template --name {name} --namespace {namespace} --values $(location {values}) $(location {chart}) > $@".format(name = release_name, namespace = namespace or "default", chart = chart, values = values),
        tools = ["@kubernetes_helm//:helm"],
    )
