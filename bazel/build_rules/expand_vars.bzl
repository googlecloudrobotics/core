"""Substitutes variables in templates.

https://docs.bazel.build/versions/master/skylark/lib/actions.html#expand_template
"""

def _expand_vars_impl(ctx):
    # Wrap all variable names into ${}
    substitutions = {"${" + key + "}": val for key, val in ctx.attr.substitutions.items()}
    substitutions["$$"] = "$"

    ctx.actions.expand_template(
        template = ctx.file.template,
        output = ctx.outputs.out,
        substitutions = substitutions,
    )

expand_vars = rule(
    implementation = _expand_vars_impl,
    attrs = {
        "template": attr.label(mandatory = True, allow_single_file = True),
        "out": attr.output(mandatory = False),
        "substitutions": attr.string_dict(mandatory = False),
    },
    output_to_genfiles = True,
)
