load("//bazel:container_push.bzl", "container_push")

def _get_runfile_path(ctx, f):
    """Return the runfiles relative path of f."""
    if ctx.workspace_name:
        return "${RUNFILES}/" + ctx.workspace_name + "/" + f.short_path
    else:
        return "${RUNFILES}/" + f.short_path

def _impl(ctx):
    runfiles = ctx.attr._sh_tpl.default_runfiles.files.to_list()
    for target in ctx.attr.push_targets:
        runfiles.append(target.files_to_run.executable)
        runfiles.extend(target.default_runfiles.files.to_list())

    ctx.actions.expand_template(
        template = ctx.file._sh_tpl,
        substitutions = {
            "%{commands}": "\n".join(
                [
                    "if [[ -z \"${TAG:-}\" ]]; then echo >&2 \"$0: TAG environment variable must be set when pushing images.\"; exit 1; fi",
                ] + [
                    "async {command} --repository=\"${{CONTAINER_REGISTRY}}/{repository}\" --tag=\"${{TAG}}\"".format(
                        command = _get_runfile_path(ctx, target.files_to_run.executable),
                        repository = repository,
                    )
                    for target, repository in zip(ctx.attr.push_targets, ctx.attr.images.keys())
                ],
            ),
        },
        output = ctx.outputs.executable,
        is_executable = True,
    )

    return [DefaultInfo(runfiles = ctx.runfiles(files = runfiles))]

_push_all = rule(
    attrs = {
        # Implicit dependencies.
        "push_targets": attr.label_list(
            allow_files = True,
        ),
        "images": attr.string_dict(
            default = {},
        ),
        "_sh_tpl": attr.label(
            default = Label("@cloud_robotics//bazel/build_rules/app_chart:push_all.sh.tpl"),
            allow_single_file = True,
        ),
    },
    executable = True,
    implementation = _impl,
)

def push_all(name, images = {}, **kwargs):
    """Creates a script to push several container images to a docker registry.
    The registry has to be specified as parameter when invoking the script.

    Args:
      images: dict. Repository names as keys and images to be pushed as values.
    """
    if "push_targets" in kwargs:
        fail("reserved for internal use by push_all macro", attr = "push_targets")

    images = images or {}
    push_targets = []
    for repository, image in images.items():
        push_target = name + "." + repository + ".push"
        push_targets.append(push_target)
        container_push(
            name = push_target,
            image = image,
        )

    _push_all(name = name, images = images, push_targets = push_targets, **kwargs)
