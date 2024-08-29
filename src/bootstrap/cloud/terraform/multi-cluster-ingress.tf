# Multi-Cluster Ingress
#
# If the config calls for additional regions, this registers them all into one
# "fleet" (aka GKE Hub). It enables multi-cluster services and multi-cluster
# ingress, so that the primary "cloud-robotics" cluster is the source-of-truth
# for gateway configuration.

resource "google_gke_hub_feature" "multi_cluster_service_discovery" {
  count = length(var.additional_regions) > 0 ? 1 : 0

  name = "multiclusterservicediscovery"
  location = "global"
  project = data.google_project.project.project_id
}

resource "google_gke_hub_feature" "multi_cluster_ingress" {
  count = length(var.additional_regions) > 0 ? 1 : 0

  name = "multiclusteringress"
  location = "global"
  project = data.google_project.project.project_id
  spec {
    multiclusteringress {
      config_membership = google_gke_hub_membership.cloud_robotics[0].id
    }
  }
}

# The GKE cluster called "cloud-robotics" is the primary cluster and the source
# for Gateway configs.
#
# Both of these memberships set location = <cluster region>. I don't know if
# this is important or if it's just Anthos metadata, but this is what
# `gcloud container fleet memberships register` does (as opposed to what its
# docs say, which is that it uses the location rather than the region of the
# cluster, which could be a zone).
resource "google_gke_hub_membership" "cloud_robotics" {
  count = length(var.additional_regions) > 0 ? 1 : 0

  membership_id = "cloud-robotics"
  project = data.google_project.project.project_id
  location = var.region
  endpoint {
    gke_cluster {
      resource_link = google_container_cluster.cloud-robotics.id
    }
  }
}

resource "google_gke_hub_membership" "cloud_robotics_ar" {
  for_each              = var.additional_regions
  project               = data.google_project.project.project_id
  membership_id         = format("%s-%s", each.key, "ar-cloud-robotics")
  location = each.value.region
  endpoint {
    gke_cluster {
      resource_link = google_container_cluster.cloud-robotics-ar[each.key].id
    }
  }
}
