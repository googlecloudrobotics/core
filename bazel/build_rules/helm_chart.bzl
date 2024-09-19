def helm_chart(ctx, name, chart, files, templates, values, version, helm, out):
    """Starlark function that builds a helm chart.

    Args:
      name: string. Must match the name in Chart.yaml.
      chart: file. The Chart.yaml file.
      files: list of non-template files to put in files/.
      templates: list of template files.
      values: file. The values.yaml file.
      version: string. Overwrites any version in Chart.yaml.
      helm: file. The Helm tool.
      out: file. The file that the chart is built to.
    """

    cmd = """
      mkdir {name} {name}/templates
      cp {chart} {name}/Chart.yaml
      cp {values} {name}/values.yaml
      """.format(name = name, chart = chart.path, values = values.path)
    if templates:
        template_files = " ".join([t.path for t in templates])

        # Use a single cp invocation to detect filename clashes.
        cmd += "cp {templates} {name}/templates\n".format(name = name, templates = template_files)
    if files:
        cmd += "mkdir {name}/files\n".format(name = name)
        files_locations = " ".join([f.path for f in files])

        # Use a single cp invocation to detect filename clashes.
        cmd += "cp {files} {name}/files\n".format(name = name, files = files_locations)
    cmd += """
      # Linter is too noisy, swallow its output when not failing
      {helm} lint --strict {name} >/dev/null 2>&1 || \\
        {helm} lint --strict {name}
      {helm} package \\
          --save=false --version={version} {name} \\
        | (grep -v "Successfully packaged" || true)
      mv $(basename {output}) {output}
      rm -rf {name}""".format(name = name, version = version, helm = helm.path, output = out.path)
    ctx.actions.run_shell(
        inputs = [chart, values] + templates + (files or []),
        tools = [helm],
        outputs = [out],
        command = cmd,
    )
