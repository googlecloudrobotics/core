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
  std_repositories = [
    { repository = "asia.gcr.io", location="asia" },
    { repository = "eu.gcr.io", location="europe" },
    { repository = "gcr.io", location="us" },
    { repository = "us.gcr.io", location="us" },
  ]
}

# import existing repos, see: gcloud artifacts repositories list --project=<project-id>
import {
  id = "projects/${data.google_project.project.project_id}/locations/asia/repositories/asia.gcr.io"
  to = google_artifact_registry_repository.gcrio_repositories[0]
}
import {
  id = "projects/${data.google_project.project.project_id}/locations/europe/repositories/eu.gcr.io"
  to = google_artifact_registry_repository.gcrio_repositories[1]
}
import {
  id = "projects/${data.google_project.project.project_id}/locations/us/repositories/gcr.io"
  to = google_artifact_registry_repository.gcrio_repositories[2]
}
import {
  id = "projects/${data.google_project.project.project_id}/locations/us/repositories/us.gcr.io"
  to = google_artifact_registry_repository.gcrio_repositories[3]
}

resource "google_artifact_registry_repository" "gcrio_repositories" {
  project       = data.google_project.project.project_id
  location      = local.std_repositories[count.index].location
  repository_id = local.std_repositories[count.index].repository
  format        = "docker"

  cleanup_policy_dry_run = false
  cleanup_policies {
    id     = "delete-untagged"
    action = "DELETE"
    condition {
      tag_state    = "UNTAGGED"
      older_than   = "30d"
    }
  }

  count         = length(local.std_repositories)
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
