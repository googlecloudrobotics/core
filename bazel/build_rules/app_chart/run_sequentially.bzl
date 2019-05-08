def _get_runfile_path(ctx, f):
    """Return the runfiles relative path of f."""
    if ctx.workspace_name:
        return "${RUNFILES}/" + ctx.workspace_name + "/" + f.short_path
    else:
        return "${RUNFILES}/" + f.short_path

def _impl(ctx):
    runfiles = ctx.attr._sh_tpl.default_runfiles.files.to_list()
    for target in ctx.attr.targets:
        runfiles.append(target.files_to_run.executable)
        runfiles.extend(target.default_runfiles.files.to_list())

    variables = "PYTHON_RUNFILES=\"${RUNFILES}\" "
    ctx.actions.expand_template(
        template = ctx.file._sh_tpl,
        substitutions = {
            "%{commands}": "\n".join([
                variables + _get_runfile_path(ctx, command.files_to_run.executable) + " \"$@\""
                for command in ctx.attr.targets
            ]),
        },
        output = ctx.outputs.executable,
        is_executable = True,
    )

    return [DefaultInfo(runfiles = ctx.runfiles(files = runfiles))]

run_sequentially = rule(
    attrs = {
        "targets": attr.label_list(
            allow_empty = True,
            mandatory = True,
        ),
        "_sh_tpl": attr.label(
            default = Label("@cloud_robotics//bazel/build_rules/app_chart:run_sequentially.sh.tpl"),
            allow_single_file = True,
        ),
    },
    executable = True,
    implementation = _impl,
)
"""Run multiple targets sequentially.

This rule builds a "bazel run" target that runs a series of subtargets in
sequence. If a subtarget has errors, execution ends early with an error.

Args:
  targets: A list of targets that can be run with "bazel run".
"""
