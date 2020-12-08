resource "google_cloudiot_registry" "cloud-robotics" {
  name   = "cloud-robotics"
  region = var.region

  depends_on = [
    google_project_service.cloudiot,

    # TODO(b/123619046): remove this dependency. It is here because the cluster
    # takes a long time to create, so by waiting until that's done, the IoT
    # Registry can be safely created without triggering the bug.
    google_container_cluster.cloud-robotics,
  ]
}
