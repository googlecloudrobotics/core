"""Macros to help rearrange files."""

def copy_files(name, srcs, outdir, visibility = None):
    """Creates copies of files within a directory.

    For each source file, a copy with the same basename is created in outdir. A
    filegroup target <name> is created containing the new files.

    Args:
      srcs: list of files.
      outdir: output directory.
      visibility: handled through to the targets.
    """
    outs = []
    for src in srcs:
        filename = src.split("/")[-1]
        out = "%s/%s" % (outdir, filename)
        native.genrule(
            name = "%s_%s" % (name, filename),
            srcs = [src],
            outs = [out],
            cmd = "cp $< $@",
            visibility = visibility,
        )
        outs.append(out)

    native.filegroup(
        name = name,
        srcs = outs,
        visibility = visibility,
    )
