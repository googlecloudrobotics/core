resource "google_container_cluster" "cloud-robotics" {
  name                  = "cloud-robotics"
  location              = var.zone
  min_master_version    = "1.16"
  enable_shielded_nodes = true
  depends_on            = [google_project_service.container]

  # We can't create a cluster with no node pool defined, but we want to only use
  # separately managed node pools. So we create the smallest possible default
  # node pool and immediately delete it.
  remove_default_node_pool = true

  initial_node_count = 1

  timeouts {
    create = "1h"
    update = "1h"
    delete = "1h"
  }
}

resource "google_container_node_pool" "cloud-robotics" {
  name     = "cloud-robotics"
  location = var.zone
  cluster  = google_container_cluster.cloud-robotics.name

  initial_node_count = 2

  autoscaling {
    min_node_count = 2
    max_node_count = 10
  }

  node_config {
    machine_type = "e2-standard-4"

    oauth_scopes = [
      "https://www.googleapis.com/auth/bigquery",
      "https://www.googleapis.com/auth/cloud-platform",
      "https://www.googleapis.com/auth/cloud-platform.read-only",
      "https://www.googleapis.com/auth/cloud.useraccounts",
      "https://www.googleapis.com/auth/compute",
      "https://www.googleapis.com/auth/datastore", # TODO: unused
      "https://www.googleapis.com/auth/devstorage.full_control",
      "https://www.googleapis.com/auth/logging.write",
      "https://www.googleapis.com/auth/monitoring",
      "https://www.googleapis.com/auth/monitoring.write",
      "https://www.googleapis.com/auth/ndev.clouddns.readwrite",
      "https://www.googleapis.com/auth/pubsub",
      "https://www.googleapis.com/auth/servicecontrol",
      "https://www.googleapis.com/auth/trace.append",
      "https://www.googleapis.com/auth/userinfo.email",
    ]
  }
}

# TODO(swolter): Depend on APIs.

