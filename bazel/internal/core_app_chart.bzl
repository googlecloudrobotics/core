load("//:config.bzl", "CLOUD_ROBOTICS_CONTAINER_REGISTRY", "DOCKER_TAG")
load("//bazel:app_chart.bzl", "app_chart")

def core_app_chart(*args, **kwargs):
    """Convenience macro that sets registry and docker_tag for app_chart."""
    app_chart(
        *args,
        registry = CLOUD_ROBOTICS_CONTAINER_REGISTRY,
        docker_tag = DOCKER_TAG,
        **kwargs
    )
