load("@cloud_robotics//bazel/build_rules/app_chart:run_parallel.bzl", "run_parallel")

def app(name, charts, v2 = False, visibility = None):
    """Macro for a standard Cloud Robotics app.

    This macro establishes two subrules for app name "foo":
    - :foo.push pushes the Docker images for the app.
    - :foo.yaml is a YAML file with the app CR that you need to push to
      Kubernetes. Use k8s_object to push it, or compile it into a Helm chart.

    Args:
      name: string. Name of the app.
      charts: list of targets. Helm charts for this app.
      visibility: Visibility.
    """
    pkg = Label("{}//{}".format(native.repository_name(), native.package_name()))
    chart_labels = [pkg.relative(c) for c in charts]
    run_parallel(
        name = name + ".push",
        targets = ["//{}:{}.push".format(c.package, c.name) for c in chart_labels],
        visibility = visibility,
    )

    if v2:
        native.genrule(
            # we name this differently than the file we produce to silence:
            #   target 'xxx.yaml' is both a rule and a file; please choose another name for the rule
            name = name + ".manifest",
            srcs = [
                "//{}:{}.snippet-v2-yaml".format(c.package, c.name)
                for c in chart_labels
                if not c.name.endswith("cloud-per-robot")
            ],
            outs = [name + ".yaml"],
            cmd = """cat - $(SRCS) > $@ <<EOF
apiVersion: apps.cloudrobotics.com/v1alpha1
kind: App
metadata:
  name: {name}-dev
spec:
  components:
EOF
""".format(name = name),
            visibility = visibility,
        )
    else:
        native.genrule(
            name = name + ".manifest",
            srcs = ["//{}:{}.snippet-yaml".format(c.package, c.name) for c in chart_labels],
            outs = [name + ".yaml"],
            cmd = """cat - $(SRCS) > $@ <<EOF
apiVersion: registry.cloudrobotics.com/v1alpha1
kind: App
metadata:
  name: {name}
spec:
  charts:
EOF
""".format(name = name),
            visibility = visibility,
        )
