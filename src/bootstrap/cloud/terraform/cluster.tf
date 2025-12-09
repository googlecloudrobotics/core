# GKE Cluster
#
# This creates a GKE cluster with Workload Identity enabled and a suitable
# service account for the nodes. This service account cannot be used by the
# workloads: see workload-identity.tf for those service accounts.

resource "google_container_cluster" "cloud-robotics" {
  project               = data.google_project.project.project_id
  name                  = "cloud-robotics"
  location              = var.cluster_type == "zonal" ? var.zone : var.region
  enable_shielded_nodes = true
  depends_on            = [google_project_service.project-services["container.googleapis.com"]]

  # TODO(ensonic): this is temporary for the zonal -> regional switch
  deletion_protection = false

  # Make the cluster VPC-native (default for v1.21+)
  networking_mode = "VPC_NATIVE"

  datapath_provider = var.datapath_provider
  # We can't create a cluster with no node pool defined, but we want to only use
  # separately managed node pools. So we create the smallest possible default
  # node pool and immediately delete it.
  remove_default_node_pool = true

  initial_node_count = 1

  addons_config {
    gke_backup_agent_config {
      enabled = true
    }
  }
  gateway_api_config {
    channel = "CHANNEL_STANDARD"
  }
  ip_allocation_policy {}
  maintenance_policy {
    recurring_window {
      # Dates specifies first ocurance, times are in UTC
      # Start late and end early to cover regions ahead/behind the sun
      # Note: we mst not make this too small, otherwise GKE cannot schedule updates
      start_time = "2025-04-05T05:00:00Z"
      end_time   = "2025-04-06T19:00:00Z"
      recurrence = "FREQ=WEEKLY;BYDAY=SA"
    }
  }
  release_channel {
    channel = "STABLE"
  }
  secret_manager_config {
    enabled = var.secret_manager_plugin
  }
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
  location              = var.cluster_type == "zonal" ? each.value.zone : each.value.region
  enable_shielded_nodes = true
  depends_on            = [google_project_service.project-services["container.googleapis.com"]]

  # TODO(ensonic): this is temporary for the zonal -> regional switch
  deletion_protection = false

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

  vertical_pod_autoscaling {
    enabled = true
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
  location = var.cluster_type == "zonal" ? var.zone : var.region
  cluster  = google_container_cluster.cloud-robotics.name

  initial_node_count = 2

  autoscaling {
    min_node_count = 1
    max_node_count = 16
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
  location = var.cluster_type == "zonal" ? each.value.zone : each.value.region
  cluster  = google_container_cluster.cloud-robotics-ar[each.key].name

  initial_node_count = 2

  autoscaling {
    min_node_count = 1
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

resource "google_project_iam_member" "gke_node_roles" {
  project = data.google_project.project.project_id
  member  = google_service_account.gke_node.member
  for_each = toset([
    # GKE recommendations
    "roles/logging.logWriter",
    "roles/monitoring.metricWriter",
    "roles/stackdriver.resourceMetadata.writer",
  ])
  role = each.key
}
