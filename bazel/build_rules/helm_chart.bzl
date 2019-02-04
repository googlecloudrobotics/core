def helm_chart(name, version, chart, values, templates, files = None, subcharts = None, visibility = None):
    """Builds a helm chart.

    Args:
      name: string. Must match the name in Chart.yaml.
      version: string. Overwrites any version in Chart.yaml.
      chart: file. The Chart.yaml file.
      value_template: file. The values.yaml file.
      templates: list of template files.
      files: list of non-template files to put in files/.
    """

    tar = ":" + name + "_files"
    out = name + "-" + version + ".tgz"
    cmd = """
      mkdir {name} {name}/templates
      cp $(location {chart}) {name}/Chart.yaml
      cp $(location {values}) {name}/values.yaml
      """.format(name = name, chart = chart, values = values)
    if templates:
        template_files = " ".join(["$(locations {t})".format(t = t) for t in templates])

        # Use a single cp invocation to detect filename clashes.
        cmd += "cp {templates} {name}/templates\n".format(name = name, templates = template_files)
    if files:
        cmd += "mkdir {name}/files\n".format(name = name)
        files_locations = " ".join(["$(locations {f})".format(f = f) for f in files])

        # Use a single cp invocation to detect filename clashes.
        cmd += "cp {files} {name}/files\n".format(name = name, files = files_locations)
    cmd += """
      # Linter is too noisy, swallow its output when not failing
      $(location @kubernetes_helm//:helm) lint --strict {name} >/dev/null 2>&1 || \\
        $(location @kubernetes_helm//:helm) lint --strict {name}
      $(location @kubernetes_helm//:helm) package \\
          --save=false --version={version} {name} \\
        | (grep -v "Successfully packaged" || true)
      mv {out} $@
      rm -rf {name}""".format(name = name, version = version, out = out)
    native.genrule(
        name = name,
        srcs = [chart, values] + templates + (files or []),
        tools = ["@kubernetes_helm//:helm"],
        outs = [out],
        cmd = cmd,
        visibility = visibility,
    )
