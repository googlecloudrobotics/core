# Container registry configuration

locals {
    service_acounts = flatten([
        "serviceAccount:${google_service_account.gke_node.email}",
        "serviceAccount:${google_service_account.human-acl.email}",
        var.onprem_federation ? ["serviceAccount:${google_service_account.robot-service[0].email}"] : [],
    ])
    private_repo_access = flatten([
        for sa in local.service_acounts : [
            for prj in var.private_image_repositories : {
                prj = prj
                sa   = sa
            }
        ]
    ])
}

resource "google_artifact_registry_repository_iam_member" "gcrio_gar_reader" {
  project    = data.google_project.project.project_id
  location   = "us"
  repository = "gcr.io"
  role       = "roles/artifactregistry.reader"
  count      = length(local.service_acounts)
  member     = local.service_acounts[count.index]
}

resource "google_artifact_registry_repository_iam_member" "private_gcrio_gar_reader" {
  location   = "us"
  repository = "gcr.io"
  role       = "roles/artifactregistry.reader"
  count      = length(local.private_repo_access)
  project    = local.private_repo_access[count.index].prj
  member     = local.private_repo_access[count.index].sa
}
