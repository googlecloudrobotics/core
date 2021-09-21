"""Debug util for repository definitions."""

def debug_repository(repo, *fields):
    """debug_repository(repo) identifies which version of a repository has been
    defined in the WORKSPACE by printing some of its fields. Example:

    # at the bottom of the WORKSPACE file
    load("//bazel:debug_repository.bzl", "debug_repository")

    debug_repository("org_golang_x_net")

    If needed, you can override the printed fields by passing additional parameters:

    debug_repository("io_grpc_grpc_java", "patches", "urls")
    """

    if len(fields) == 0:
        fields = ["branch", "commit", "tag", "url", "urls"]

    rule = native.existing_rule(repo)
    if rule == None:
        print(repo, "not found")
        return

    for f in fields:
        if f in rule and len(rule[f]) > 0:
            print(repo, f, rule[f])
