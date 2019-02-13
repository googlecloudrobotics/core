def _impl(ctx):
    # The list of arguments we pass to the script.
    args = [
        "--output",
        ctx.outputs.yaml_file.path,
        "--proto_descriptor_file",
        ctx.file.descriptor.path,
        "--message",
        ctx.attr.message,
        "--group",
        ctx.attr.group,
        "--plural",
        ctx.attr.plural,
    ]
    if ctx.attr.namespaced:
        args.append("--namespaced")

    # Action to call the script.
    ctx.actions.run(
        inputs = [ctx.file.descriptor],
        outputs = [ctx.outputs.yaml_file],
        arguments = args,
        progress_message = "Generating CRD into %s" % ctx.outputs.yaml_file.short_path,
        executable = ctx.executable._crd_generator_tool,
    )

proto_crd = rule(
    implementation = _impl,
    attrs = {
        "descriptor": attr.label(
            allow_single_file = True,
            doc = "Proto descriptor with the CRD's definition.",
        ),
        "message": attr.string(
            doc = "Fully qualified name of the proto message that defines the CRD",
            mandatory = True,
        ),
        "group": attr.string(
            doc = "Kubernetes API group without version",
            mandatory = True,
        ),
        "plural": attr.string(
            doc = "Plural form, defaults to message basename + s if omitted",
            default = "",
        ),
        "namespaced": attr.bool(
            doc = "Make CRD namespaced or cluster-scoped",
            default = True,
        ),
        "_crd_generator_tool": attr.label(
            executable = True,
            cfg = "host",
            allow_files = True,
            default = Label("@cloud_robotics//src/go/cmd/crd-generator"),
        ),
    },
    outputs = {"yaml_file": "%{name}.yaml"},
)

"""Generates a CRD YAML file for a proto-defined CRD.

Args:
  name: string.
  descriptor: label. Proto descriptor with the CRD's definition.
  group: string. Kubernetes API group without version.
  plural: string. Defaults to message basename + "s" if omitted.
  namespaced: bool. Defaults to true. If false, CRD is cluster-scoped.
  visibility: Visibility.
"""
