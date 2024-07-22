# Container registry configuration

locals {
    service_acounts = [
        "serviceAccount:${google_service_account.gke_node.email}",
        "serviceAccount:${google_service_account.human-acl.email}",
        "serviceAccount:${google_service_account.robot-service.email}",
    ]
    private_repo_access = distinct(flatten([
        for sa in local.service_acounts : [
            for prj in var.private_image_repositories : {
                prj = prj
                sa   = sa
            }
        ]
    ]))
}

resource "google_artifact_registry_repository_iam_member" "gcrio_gar_reader" {
  project    = data.google_project.project.project_id
  location   = "us"
  repository = "gcr.io"
  role       = "roles/artifactregistry.reader"
  for_each   = toset(local.service_acounts)
  member     = each.key
}

resource "google_artifact_registry_repository_iam_member" "private_gcrio_gar_reader" {
  for_each   = { for entry in local.private_repo_access: "${entry.sa}.${entry.prj}" => entry }
  location   = "us"
  repository = "gcr.io"
  role       = "roles/artifactregistry.reader"
  project    = each.value.prj
  member     = each.value.sa
}
