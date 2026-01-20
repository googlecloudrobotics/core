# Container registry configuration

locals {
  service_acounts = flatten([
    google_service_account.gke_node.member,
    google_service_account.human-acl.member,
    var.onprem_federation ? [google_service_account.robot-service[0].member] : [],
  ])
  # TODO: use the regional repos depending on settings in the future
  private_repo_access = flatten([
    for sa in local.service_acounts : [
      for prj in var.private_image_repositories : {
        prj = prj
        sa  = sa
      }
    ]
  ])
  std_repositories = {
    "asia.gcr.io" = { location = "asia" }
    "eu.gcr.io"   = { location = "europe" }
    "gcr.io"      = { location = "us" }
    "us.gcr.io"   = { location = "us" }
  }
}

# import existing repos, see: gcloud artifacts repositories list --project=<project-id>
# sadly the import statement needs to be commented out when creating a new project:
# https://github.com/hashicorp/terraform/issues/33633
import {
  for_each = local.std_repositories

  id = "projects/${data.google_project.project.project_id}/locations/${each.value.location}/repositories/${each.key}"
  to = google_artifact_registry_repository.gcrio_repositories[each.key]
}

resource "google_artifact_registry_repository" "gcrio_repositories" {
  for_each = local.std_repositories

  project       = data.google_project.project.project_id
  location      = each.value.location
  repository_id = each.key
  format        = "docker"

  cleanup_policy_dry_run = false
  cleanup_policies {
    id     = "delete-untagged"
    action = "DELETE"
    condition {
      tag_state  = "UNTAGGED"
      older_than = "30d"
    }
  }
}

resource "google_artifact_registry_repository_iam_member" "gcrio_gar_reader" {
  project    = data.google_project.project.project_id
  location   = "us"
  repository = "gcr.io"
  role       = "roles/artifactregistry.reader"
  count      = length(local.service_acounts)
  member     = local.service_acounts[count.index]
  depends_on = [ google_artifact_registry_repository.gcrio_repositories ]
}

resource "google_artifact_registry_repository_iam_member" "private_gcrio_gar_reader" {
  location   = "us"
  repository = "gcr.io"
  role       = "roles/artifactregistry.reader"
  count      = length(local.private_repo_access)
  project    = local.private_repo_access[count.index].prj
  member     = local.private_repo_access[count.index].sa
  depends_on = [ google_artifact_registry_repository.gcrio_repositories ]
}
