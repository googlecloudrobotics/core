resource "google_container_cluster" "cloud-robotics" {
  name               = "cloud-robotics"
  zone               = "${var.zone}"
  min_master_version = "1.11"
  enable_legacy_abac = true
  depends_on         = ["google_project_service.container"]

  node_pool {
    initial_node_count = 1

    autoscaling = {
      min_node_count = 1
      max_node_count = 10
    }

    node_config {
      machine_type = "n1-standard-8"

      oauth_scopes = [
        "https://www.googleapis.com/auth/bigquery",
        "https://www.googleapis.com/auth/cloud-platform",
        "https://www.googleapis.com/auth/cloud-platform.read-only",
        "https://www.googleapis.com/auth/cloud.useraccounts",
        "https://www.googleapis.com/auth/compute",
        "https://www.googleapis.com/auth/datastore",
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

  timeouts {
    create = "1h"
    update = "1h"
    delete = "1h"
  }
}

# TODO(swolter): Depend on APIs.

