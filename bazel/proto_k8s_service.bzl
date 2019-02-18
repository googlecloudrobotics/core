load("@cloud_robotics//bazel/build_rules:expand_vars.bzl", "expand_vars")
load("@cloud_robotics//bazel/build_rules:proto_descriptor.bzl", "proto_descriptor")
load("@cloud_robotics//bazel:proto_crd.bzl", "proto_crd")

# TODO(#3): We need to add a dependency for @grpc_ecosystem_grpc_gateway to repositories.bzl.
load("@grpc_ecosystem_grpc_gateway//protoc-gen-swagger:defs.bzl", "protoc_gen_swagger")

def _impl(ctx):
    import_files = [f.path for f in ctx.attr.spec.proto.direct_sources]
    imports = ["import \"{}\";".format(f) for f in import_files]
    options = ["option {} = \"{}\";".format(k, v) for k, v in ctx.attr.file_options.items()]

    ctx.actions.expand_template(
        template = ctx.file._template,
        output = ctx.outputs.proto_file,
        substitutions = {
            "${OPTIONS}": "\n".join(options),
            "${KIND}": ctx.attr.message.rsplit(".", 1)[1],
            "${NAMESPACE}": ctx.attr.message.rsplit(".", 1)[0],
            "${IMPORTS}": "\n".join(imports),
        },
    )

proto_k8s_service_proto = rule(
    implementation = _impl,
    attrs = {
        "_template": attr.label(
            default = "@cloud_robotics//bazel:proto_k8s_service.proto.tmpl",
            allow_single_file = True,
            mandatory = False,
        ),
        "message": attr.string(mandatory = True),
        "file_options": attr.string_dict(mandatory = True),
        "spec": attr.label(mandatory = True, allow_files = True),
    },
    output_to_genfiles = True,
    outputs = {"proto_file": "%{name}.proto"},
)

def proto_k8s_service(name, message, group, spec, file_options, visibility = None):
    """Generates a proto service for the K8s adapter.

    This rule generates a :foo proto_library rule with the proto service, and
    a foo_proto_crd.yaml file with a Kubernetes CRD definition.

    TODO(swolter): This rule could be refactored to work with Gazelle if Gazelle
    read imports from generated files
    (https://github.com/bazelbuild/bazel-gazelle/issues/453). We'd have to
    add an explicit arg "out" that would contain the generated proto file.

    Args:
      message: string. Fully qualified name of the message.
      group: string. Kubernetes API group for the CRD.
      spec: label. a proto_library rule with `message`Spec and `message`Status.
      file_options: dict of string. File-level options for the generated
        proto file.
    """
    proto_k8s_service_proto(
        name = name + "_service_genproto",
        message = message,
        spec = spec,
        file_options = file_options,
    )

    native.proto_library(
        name = name,
        srcs = [
            name + "_service_genproto",
        ],
        visibility = visibility,
        deps = [
            spec,
            "//third_party/kubernetes_proto/meta:meta_proto",
        ],
    )

    proto_descriptor(
        name = name + "_proto_descriptor",
        out = name + "_proto_descriptor.pb",
        proto_library = name,
    )

    protoc_gen_swagger(
        name = name + "_swagger",
        proto = name,
    )

    proto_crd(
        name = name + "_proto_crd",
        descriptor = name + "_proto_descriptor",
        group = group,
        message = message,
        openapi_spec = name + "_swagger",
        visibility = visibility,
    )
