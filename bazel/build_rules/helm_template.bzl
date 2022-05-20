def helm_template(name, release_name, chart, values, namespace = None, helm_version = 2):
    """Locally expand a helm chart.

    Args:
      chart: build label, referencing the chart to expand.
      values: label. File with expand-time values.
    """

    tool = ""
    cmd = ""
    if helm_version == 2:
        tool = "@kubernetes_helm//:helm"
        cmd = "$(location {tool}) template --name {name} --namespace {namespace} --values $(location {values}) $(location {chart}) > $@".format(name = release_name, namespace = namespace or "default", chart = chart, tool = tool, values = values)
    elif helm_version == 3:
        tool = "@kubernetes_helm3//:helm"
        cmd = "$(location {tool}) template {name} $(location {chart}) --namespace {namespace} --values $(location {values}) > $@".format(name = release_name, namespace = namespace or "default", chart = chart, tool = tool, values = values)
    else:
        fail("Unsupported helm version. Expected {2,3}, got ", helm_version)

    native.genrule(
        name = name,
        srcs = [chart, values],
        outs = [name + ".yaml"],
        cmd = cmd,
        tools = [tool],
    )
