"""Build macro for ROS plugins."""

def ros_plugins(
        name,
        srcs,
        description,
        lib,
        manifest,
        rospkg,
        deps,
        visibility = None,
        copts = None):
    """Makes ROS plugins available for static linking.

    The plugins can be linked into a cc_binary by adding :<name>_srcs to the srcs
    and adding :<name>_deps to the deps.

    At runtime, the binary will need to be able to find the package directory. It
    will also need `catkin_find` available on the PATH.  With rosenv, this would
    mean:
      re.add_package("<repo name>/<rospkg>")
      re.add_to_path("com_github_ros_catkin")

    Args:
      srcs: List of C++ sources for the plugins. These are the files that use
            PLUGINLIB_EXPORT_CLASS.
      lib: Name of the .so file containing the plugins (as pointed to by the
           plugin description file).
      manifest: package.xml file.
      rospkg: Name of the ROS package.
      description: Plugin description file (as pointed to by the manifest).
      deps: Build dependencies of the plugin sources.
      visibility: Will be handed through to the synthesized targets.
      copts: Will be handed through to the C++ target.
    """
    copts = copts or []

    if not srcs:
        fail("must not be empty", attr = "srcs")

    # We assume that the package.xml is in a subdirectory with the name of the
    # ROS package. This is the pattern followed by ros-planning/navigation.
    # TODO(rodrigoq): if we need to support other layouts, we could copy files to
    # the expected layout.
    if manifest != rospkg + "/package.xml":
        fail("expected manifest = '%s/package.xml'" % rospkg)

    lib_path = rospkg + "/" + lib

    # Dummy plugin library (plugins are statically included)
    native.cc_binary(
        name = lib_path,
        linkshared = 1,
    )

    native.filegroup(
        name = name + "_srcs",
        srcs = srcs,
    )

    native.cc_library(
        name = name + "_deps",
        visibility = visibility,
        copts = copts,
        data = [
            description,
            lib_path,
            manifest,
            "@com_github_ros_catkin//:catkin_find",
        ],
        deps = deps,
    )
