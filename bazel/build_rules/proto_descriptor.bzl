# Rule to generate the proto file descriptor used in deployment of Kubernetes
# services.

def paths(files):
    return [f.path for f in files]

def _impl(ctx):
    descriptors = ctx.attr.proto_library.proto.transitive_descriptor_sets
    ctx.actions.run_shell(
        inputs = descriptors,
        outputs = [ctx.outputs.out],
        command = "cat %s > %s" % (
            " ".join(paths(descriptors)),
            ctx.outputs.out.path,
        ),
    )

proto_descriptor = rule(
    implementation = _impl,
    attrs = {
        "proto_library": attr.label(),
        "out": attr.output(mandatory = True),
    },
)
