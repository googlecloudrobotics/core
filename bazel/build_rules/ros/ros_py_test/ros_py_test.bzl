"""Support for Python rostest scripts."""

# _expand_template is used to generate boilerplate files required by rostest.
def _expand_template_impl(ctx):
    ctx.actions.expand_template(
        template = ctx.file.template,
        output = ctx.outputs.out,
        substitutions = ctx.attr.substitutions,
    )

_expand_template = rule(
    implementation = _expand_template_impl,
    attrs = {
        "template": attr.label(mandatory = True, allow_single_file = True),
        "substitutions": attr.string_dict(mandatory = True),
        "out": attr.output(mandatory = True),
    },
    output_to_genfiles = True,
)

def ros_py_test(name, srcs, deps = None, main = None, rospkg = None, size = None):
    """Runs a Python test script with rostest.

    This can be used for tests that require a roscore. The test script will be
    built into a py_binary that is started by roslaunch. It should use
    `rostest.rosrun` to be compatible with rostest.

    TODO(rodrigoq): make this more flexible:
    - support C++ tests
    - custom package.xml
    - custom test launch file

    Args:
      srcs: source files for py_binary
      deps: dependencies for py_binary
      main: main file for py_binary (default: `name`)
      rospkg: name of the ROS package (default: `name`)
      size: as for py_test
    """

    main = main or name + ".py"
    rospkg = rospkg or name

    if not srcs:
        fail("must not be empty", attr = "srcs")

    test_harness_name = name + "_harness.py"
    test_launch_name = name + ".test"
    test_node_name = name + "_node"
    substitutions = {
        "${NAME}": name,
        "${ROSPKG}": rospkg,
    }

    # Generate files and targets for rostest. The structure is as follows:
    #
    # - test harness: (generated)
    #   The test harness is a Python script that sets up the environment with
    #   rosenv and then starts rostest with the test launch file, which it gets
    #   on the command line.
    #
    # - test launch file: (generated)
    #   The test launch file is a roslaunch XML file that tells rostest to start
    #   the test node.
    #
    # - test node: (supplied by user)
    #   The test node is a Python script that runs tests in the environment
    #   defined by the test launch file.
    #
    # - package.xml: (generated)
    #   Required by roslaunch to find the test node.

    _expand_template(
        name = "generate_package.xml",
        template = "@cloud_robotics//bazel/build_rules/ros/ros_py_test:package.xml.tpl",
        out = "package.xml",
        substitutions = substitutions,
    )

    _expand_template(
        name = "generate_" + test_harness_name,
        template = "@cloud_robotics//bazel/build_rules/ros/ros_py_test:harness.py.tpl",
        out = test_harness_name,
        substitutions = substitutions,
    )

    _expand_template(
        name = "generate_" + test_launch_name,
        template = "@cloud_robotics//bazel/build_rules/ros/ros_py_test:template.test.tpl",
        out = test_launch_name,
        substitutions = substitutions,
    )

    native.py_binary(
        name = test_node_name,
        srcs = srcs,
        main = main,
        deps = deps,
    )

    # TODO(rodrigoq): allow concurrent running of rostests and remove tags =
    # ["exclusive"]. Currently this is not supported upstream:
    # https://github.com/ros/ros_comm/issues/1359
    native.py_test(
        name = name,
        size = size,
        srcs = [test_harness_name],
        main = test_harness_name,
        args = [
            "$(location :%s)" % test_node_name,
            "$(location :%s)" % test_launch_name,
        ],
        tags = ["exclusive"],
        data = [
            ":package.xml",
            ":" + test_node_name,
            ":" + test_launch_name,
        ],
        deps = [
            "@//src/python/rosenv",
            "@com_github_ros_ros_comm//:rostest_lib",
        ],
    )
