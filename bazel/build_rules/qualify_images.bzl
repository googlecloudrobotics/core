def qualify_images(images, registry, docker_tag):
    """This function fully qualifies image basenames in the dict.

    Args:
      images: A dict from image basename ("cloud-master") to build target.
      registry: The Docker registry basename (gcr.io/my-project).
      docker_tag: A docker tag ("latest").
    Returns:
      A dict from fully qualified docker reference
      ("eu.gcr.io/my-project/cloud-master:latest") to build target.
    """
    return {
        registry + "/" + key + ":" + docker_tag: value
        for key, value in images.items()
    }
