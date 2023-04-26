# GKE Cluster
#
# This creates a GKE cluster with Workload Identity enabled and a suitable
# service account for the nodes. This service account cannot be used by the
# workloads: see workload-identity.tf for those service accounts.

resource "google_container_cluster" "cloud-robotics" {
  project               = data.google_project.project.project_id
  name                  = "cloud-robotics"
  location              = var.zone
  min_master_version    = "1.22"
  enable_shielded_nodes = true
  depends_on            = [google_project_service.container]

  # Make the cluster VPC-native (default for v1.21+)
  networking_mode = "VPC_NATIVE"
  ip_allocation_policy {}

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

  workload_identity_config {
    workload_pool = "${data.google_project.project.project_id}.svc.id.goog"
  }
}

resource "google_container_cluster" "cloud-robotics-ar" {
  for_each              = var.additional_regions
  project               = data.google_project.project.project_id
  name                  = format("%s-%s", each.key, "ar-cloud-robotics")
  location              = each.value.zone
  min_master_version    = "1.22"
  enable_shielded_nodes = true
  depends_on            = [google_project_service.container]

  # Make the cluster VPC-native (default for v1.21+)
  networking_mode = "VPC_NATIVE"
  ip_allocation_policy {}

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

  workload_identity_config {
    workload_pool = "${data.google_project.project.project_id}.svc.id.goog"
  }
}

# This node pool uses Workload Identity and a non-default service account, as
# recommended for improved security.
resource "google_container_node_pool" "cloud_robotics_base_pool" {
  project  = data.google_project.project.project_id
  name     = "base-pool"
  location = var.zone
  cluster  = google_container_cluster.cloud-robotics.name

  initial_node_count = 2

  autoscaling {
    min_node_count = 2
    max_node_count = 10
  }

  node_config {
    machine_type = "e2-standard-4"
    # The GKE Metadata Server enables Workload Identity.
    workload_metadata_config {
      mode = "GKE_METADATA"
    }
    service_account = google_service_account.gke_node.email
    oauth_scopes = [
      "https://www.googleapis.com/auth/cloud-platform",
      "https://www.googleapis.com/auth/userinfo.email",
    ]
  }
}

resource "google_container_node_pool" "cloud_robotics_base_pool_ar" {
  for_each = var.additional_regions
  project  = data.google_project.project.project_id
  name     = format("%s-%s", "base-pool-ar", each.key)
  location = each.value.zone
  cluster  = google_container_cluster.cloud-robotics-ar[each.key].name

  initial_node_count = 2

  autoscaling {
    min_node_count = 2
    max_node_count = 10
  }

  node_config {
    machine_type = "e2-standard-4"
    # The GKE Metadata Server enables Workload Identity.
    workload_metadata_config {
      mode = "GKE_METADATA"
    }
    service_account = google_service_account.gke_node.email
    oauth_scopes = [
      "https://www.googleapis.com/auth/cloud-platform",
      "https://www.googleapis.com/auth/userinfo.email",
    ]
  }
}

# These bindings are based on:
# https://cloud.google.com/kubernetes-engine/docs/how-to/hardening-your-cluster#use_least_privilege_sa
resource "google_service_account" "gke_node" {
  account_id   = "gke-node"
  display_name = "gke-node"
}

resource "google_project_iam_member" "gke_node_monitoring_viewer" {
  project = data.google_project.project.project_id
  role    = "roles/monitoring.viewer"
  member  = "serviceAccount:${google_service_account.gke_node.email}"
}

resource "google_project_iam_member" "gke_node_monitoring_metricWriter" {
  project = data.google_project.project.project_id
  role    = "roles/monitoring.metricWriter"
  member  = "serviceAccount:${google_service_account.gke_node.email}"
}

resource "google_project_iam_member" "gke_node_logging_logWriter" {
  project = data.google_project.project.project_id
  role    = "roles/logging.logWriter"
  member  = "serviceAccount:${google_service_account.gke_node.email}"
}

resource "google_project_iam_member" "gke_node_stackdriver_writer" {
  project = data.google_project.project.project_id
  role    = "roles/stackdriver.resourceMetadata.writer"
  member  = "serviceAccount:${google_service_account.gke_node.email}"
}

resource "google_project_iam_member" "gke_node_storage_objectViewer" {
  project = data.google_project.project.project_id
  role    = "roles/storage.objectViewer"
  member  = "serviceAccount:${google_service_account.gke_node.email}"
}

# This binding allows access to private container registries.
resource "google_project_iam_member" "gke_node_container_registry_access" {
  project = var.private_image_repositories[count.index]
  count   = length(var.private_image_repositories)
  role    = "roles/storage.objectViewer"
  member  = "serviceAccount:${google_service_account.gke_node.email}"
}
