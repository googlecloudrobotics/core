"""Generate a Java object for a Kubernetes Custom Resource."""

def _impl(ctx):
    ctx.actions.expand_template(
        template = ctx.file._template,
        output = ctx.outputs.source_file,
        substitutions = {
            "{JAVA_PACKAGE}": ".".join(reversed(ctx.attr.group.split("."))),
            "{KUBERNETES_GROUP}": ctx.attr.group,
            "{VERSION}": ctx.attr.version,
            "{KIND}": ctx.attr.name,
            "{PLURAL}": (ctx.attr.name + "s").lower(),
            "{SINGULAR}": ctx.attr.name.lower(),
        },
    )

custom_resource_java = rule(
    implementation = _impl,
    attrs = {
        "group": attr.string(mandatory = True),
        "version": attr.string(mandatory = True),
        "_template": attr.label(
            default = Label("//bazel/build_rules:CustomResource.java.template"),
            allow_single_file = True,
        ),
    },
    outputs = {"source_file": "%{name}.java"},
)
