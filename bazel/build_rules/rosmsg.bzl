"""Code generation support for ROS messages, services and actions."""

# Generate headers within this directory.
OUTPUT_PREFIX = "__rosmsg__"

# The {{ and }} are python formatted into single { and }
# See also https://docs.python.org/2/library/string.html#formatspec
_COMMON_CMD_CODE = """
# Find all directories with *.msg *.srv files. The names of these directories
# are assumed to be the package names for the inclusion search paths.
# _CopyMsgSrvFiles ensures all such files are within genfiles, although
# depending on the execution settings they may be found in the host or target
# genfiles.
# TODO(rodrigoq): use aspects for the include directories instead of find(1).
ROS_INCLUDE_DIRS=( $$(find $(GENDIR) bazel-out/host/genfiles \\
    \( -name "*.msg" -or -name "*.srv" \) \\
    -exec dirname {{}} \; | sort -u) )

# Construct the arguments for the generation scripts.
ROS_INCLUDE_FLAGS=( )
for e in "$${{ROS_INCLUDE_DIRS[@]}}"; do
  ROSPKG=$$(basename $$(dirname $${{e}}))
  ROS_INCLUDE_FLAGS[$${{#ROS_INCLUDE_FLAGS[*]}}]="-I$$ROSPKG:$$e"
done

# Add the argument for the package name.
ROS_FLAGS="-p{rospkg} $${{ROS_INCLUDE_FLAGS[@]}}"
"""

_CPP_CMD_TEMPLATE = _COMMON_CMD_CODE + """
# Run the generation script for each .msg and .srv.
for i in $(SRCS); do
  $(location @com_github_ros_gencpp//:gencpp) \\
    $$ROS_FLAGS \\
    -o {output_dir} \\
    -e $$(dirname $(location @com_github_ros_gencpp//:msg_template)) \\
    $$i
done"""

_PY_CMD_TEMPLATE = _COMMON_CMD_CODE + """
# Run the generation script for each .msg. This generates
# some_msgs/msg/_MsgName.py.
for i in $$(find $(SRCS) -name '*.msg'); do
  $(location @com_github_ros_genpy//:genmsg_py) \\
    -o {output_dir}/msg \\
    $$ROS_FLAGS $$i
done

# Now call the script with --initpy, which will generate
# some_msgs/msg/__init__.py.
$(location @com_github_ros_genpy//:genmsg_py) \\
    -o {output_dir}/msg \\
    $$ROS_FLAGS --initpy

# Same dance again for srv.
for i in $$(find $(SRCS) -name '*.srv'); do
  $(location @com_github_ros_genpy//:gensrv_py) \\
    -o {output_dir}/srv \\
    $$ROS_FLAGS $$i
done
$(location @com_github_ros_genpy//:gensrv_py) \\
    -o {output_dir}/srv \\
    $$ROS_FLAGS --initpy

# Finally, turn some_msgs/ into a Python namespace.
cat > {output_dir}/__init__.py << EOF
try:
    import pkg_resources
    pkg_resources.declare_namespace(__name__)
except ImportError:
    import pkgutil
    __path__ = pkgutil.extend_path(__path__, __name__)
EOF
"""

def _PackagePath():
    """Returns a workspace-relative path for the package."""
    repo_name = native.repository_name().lstrip("@")
    if repo_name:
        return "external/%s/%s" % (repo_name, native.package_name())
    else:
        return native.package_name()

def _Dirname(path):
    """Returns the parent directory of path."""
    i = path.rfind("/") + 1
    head = path[:i]
    if head and head != "/" * len(head):
        head = head.rstrip("/")
    return head

def _Basename(path):
    """Returns the final component of a pathname."""
    i = path.rfind("/") + 1
    return path[i:]

def _Stem(path):
    """Returns the final component of a pathname after removing the extension."""
    return _Basename(path).rsplit(".", 1)[0]

def _SplitLabel(label):
    """Returns a tuple of a short form label and a relative label.

    Args:
      label: The label as given in the BUILD file, e.g. //depot/blub, or :library

    Returns:
      A tuple of a short form label and a relative label for the supplied label.
      The short form label might be empty meaning. Joining the tuple with a ":"
      will result in a label equivalent to the supplied label.
    """
    if ":" not in label:
        label_name = _Basename(label)
        label = ":".join((label, label_name))
    label_path, label_name = label.split(":")
    return label_path, label_name

def _CopyMsgSrvFiles(name, srcs, output_dir):
    """Copy .msg and .srv files to the output directory.

    This makes sure they're accessible at the path expected by ROS.
    """
    results = []
    for src in srcs:
        item = _Basename(src)
        extension = src[-3:]
        rule_name = "_%s_%s_%s_copy" % (name, extension, _Stem(src))
        new_path = "%s/%s/%s" % (output_dir, extension, item)
        native.genrule(
            name = rule_name,
            srcs = [src],
            outs = [new_path],
            cmd = "cp $< $@",
        )
        results.append(new_path)
    return results

def _CopyHeaders(name, srcs, output_dir):
    """Copy .h files to the output directory.

    This makes sure they're accessible at the path expected by cc_library.
    """
    results = []
    for src in srcs:
        item = _Basename(src)
        rule_name = "_%s_%s_h_copy" % (name, _Stem(src))
        new_path = "%s/%s" % (output_dir, item)
        native.genrule(
            name = rule_name,
            srcs = [src],
            outs = [new_path],
            cmd = "cp $< $@",
        )
        results.append(new_path)
    return results

def _MsgsFromActions(name, srcs, output_dir, template_arguments):
    msgs = []
    for src in srcs:
        stem = _Stem(src)
        rule_name = "_%s_%s_message_generation" % (name, stem)
        new_path = "%s/msg/%s" % (output_dir, stem)
        rule_msgs = [
            "%sAction.msg" % new_path,
            "%sGoal.msg" % new_path,
            "%sActionGoal.msg" % new_path,
            "%sResult.msg" % new_path,
            "%sActionResult.msg" % new_path,
            "%sFeedback.msg" % new_path,
            "%sActionFeedback.msg" % new_path,
        ]
        native.genrule(
            name = rule_name,
            srcs = [src],
            outs = rule_msgs,
            tools = [
                "@com_github_ros_common_msgs//:genaction",
            ],
            cmd = """
$(location @com_github_ros_common_msgs//:genaction) \\
-o {output_dir}/msg $(SRCS) {rospkg} \\
| ( grep -v "Generating for action" || true ) """.format(**template_arguments),
            message = "Generating messages for action %s." % src,
        )
        msgs += rule_msgs
    return msgs

def _SynthesizePythonTargets(
        name,
        srcs,
        output_dir,
        template_arguments,
        deps,
        data,
        visibility):
    """Generate the blaze targets for python messages."""
    pylib_deps = []
    for dep in deps:
        dep_path, dep_rospkg = _SplitLabel(dep)
        pylib_deps += ["%s:%s_py" % (dep_path, dep_rospkg)]

    msg_or_srv_dirname = _Basename(_Dirname(srcs[0]))
    if msg_or_srv_dirname not in ["msg", "srv"]:
        fail("%s should be in a directory called 'msg' or 'srv', not '%s'" % (
            srcs[0],
            msg_or_srv_dirname,
        ))

    generated_files = depset([output_dir + "/__init__.py"])
    for src in srcs:
        extension = src[-3:]
        init_file = "%s/%s/__init__.py" % (output_dir, extension)
        py_file = "%s/%s/_%s.py" % (output_dir, extension, _Stem(src))
        generated_files = generated_files.union([init_file, py_file])
    generated_files = list(generated_files)

    native.genrule(
        name = "_%s_message_generation_for_python" % name,
        srcs = srcs,
        outs = generated_files,
        tools = [
            "@com_github_ros_genpy//:genmsg_py",
            "@com_github_ros_genpy//:gensrv_py",
            ":_%s_msgs" % name,
        ],
        cmd = _PY_CMD_TEMPLATE.format(**template_arguments),
        message = "Generating %s for Python." % name,
    )

    native.py_library(
        name = "%s_py" % name,
        visibility = visibility,
        srcs = [
            ":_%s_message_generation_for_python" % name,
        ],
        data = data,
        imports = [OUTPUT_PREFIX],
        deps = [
            "@com_github_ros_genpy//:genpy",
        ] + pylib_deps,
    )

def _SynthesizeCppTargets(
        name,
        srcs,
        output_dir,
        template_arguments,
        deps,
        hdrs,
        data,
        visibility,
        copts):
    """Generate the blaze targets for C++ messages."""
    cc_library_deps = []
    for dep in deps:
        dep_path, dep_rospkg = _SplitLabel(dep)
        cc_library_deps += ["%s:%s_cpp" % (dep_path, dep_rospkg)]

    generated_headers = []
    for src in srcs:
        item_path_prefix = output_dir + "/" + _Stem(src)
        generated_headers.append(item_path_prefix + ".h")
        if src.endswith(".srv"):
            generated_headers.append(item_path_prefix + "Request.h")
            generated_headers.append(item_path_prefix + "Response.h")

    native.genrule(
        name = "_%s_message_generation_for_cpp" % name,
        srcs = srcs,
        outs = generated_headers,
        tools = [
            "@com_github_ros_gencpp//:gencpp",
            "@com_github_ros_gencpp//:msg_template",
            "@com_github_ros_gencpp//:srv_template",
            ":_%s_msgs" % name,
        ],
        cmd = _CPP_CMD_TEMPLATE.format(**template_arguments),
        message = "Generating %s for C++." % name,
    )

    native.cc_library(
        name = "%s_cpp" % name,
        visibility = visibility,
        deps = [
            "@com_github_ros_roscpp_core//:cpp_common",
            "@com_github_ros_roscpp_core//:roscpp_serialization",
            "@com_github_ros_roscpp_core//:roscpp_traits",
            "@com_github_ros_std_msgs//:builtins",
        ] + cc_library_deps,
        srcs = [
            ":_%s_message_generation_for_cpp" % name,
        ],
        hdrs = generated_headers + hdrs,
        copts = copts,
        data = data,
        features = ["-layering_check"],
        linkstatic = 1,
        strip_include_prefix = OUTPUT_PREFIX,
    )

def rosmsg(
        name,
        srcs,
        rospkg = None,
        deps = None,
        hdrs = None,
        data = None,
        visibility = None,
        copts = None):
    """Generates Python and C++ libraries for ROS messages and services.

    The C++ target will be called <name>_cpp, the Python target <name>_py. A
    filegroup target _<name>_msgs is also created containing the .msg and .srv
    files for this package and for those it depends on.

    The .msg and .srv files must be located in subdirectories called `msg` or
    `srv` respectively. The resulting libraries can be imported from a path based
    on the ROS package name, eg when `rospkg = "std_msgs"`:
      C++: #include <std_msgs/String.h>
      Python: from std_msgs.msg import String

    Args:
      srcs: List of *.msg and *.srv files.
      rospkg: name of the package (defaults to the rule name).
      deps: List of other ROS messages these depend on, e.g.
            @com_github_ros_std_msgs//:std_msgs.
      hdrs: Extra list of headers that should be part of the C++ target. Rarely
            needed.
      data: Will be handed through to the synthesized libraries.
      visibility: Will be handed through to the synthesized targets.
      copts: Will be handed through to the C++ target.
    """
    rospkg = rospkg or name
    deps = deps or []
    hdrs = hdrs or []
    data = data or []
    copts = copts or []

    if not srcs:
        fail("must not be empty", attr = "srcs")

    filegroup_deps = []
    for dep in deps:
        dep_path, dep_rospkg = _SplitLabel(dep)
        filegroup_deps += ["%s:_%s_msgs" % (dep_path, dep_rospkg)]

    action_srcs = [src for src in srcs if src.endswith(".action")]
    msg_srv_srcs = [
        src
        for src in srcs
        if src.endswith(".msg") or src.endswith(".srv")
    ]

    output_dir = OUTPUT_PREFIX + "/" + rospkg

    template_arguments = {
        "rospkg": rospkg,
        "output_dir": "$(GENDIR)/%s/%s" % (_PackagePath(), output_dir),
    }

    msg_srv_srcs = _CopyMsgSrvFiles(name, msg_srv_srcs, output_dir)
    msg_srv_srcs += _MsgsFromActions(name, action_srcs, output_dir, template_arguments)

    native.filegroup(
        name = "_%s_msgs" % name,
        srcs = msg_srv_srcs + filegroup_deps,
    )

    hdrs = _CopyHeaders(name, hdrs, output_dir)
    _SynthesizeCppTargets(
        name,
        msg_srv_srcs,
        output_dir,
        template_arguments,
        deps,
        hdrs,
        data,
        visibility,
        copts,
    )
    _SynthesizePythonTargets(
        name,
        msg_srv_srcs,
        output_dir,
        template_arguments,
        deps,
        data,
        visibility,
    )
