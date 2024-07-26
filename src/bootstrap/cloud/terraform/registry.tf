# Container registry configuration

locals {
    service_acounts = [
        "serviceAccount:${google_service_account.gke_node.email}",
        "serviceAccount:${google_service_account.human-acl.email}",
        "serviceAccount:${google_service_account.robot-service.email}",
    ]
    gar_roles = [
        "roles/artifactregistry.reader",
        "roles/artifactregistry.writer",
    ]
    own_repo_access = distinct(flatten([
        for sa in local.service_acounts : [
            for role in local.gar_roles : {
                sa   = sa
                role = role
            }
        ]
    ]))
    private_repo_access = distinct(flatten([
        for sa in local.service_acounts : [
            for role in local.gar_roles : [
                for prj in var.private_image_repositories : {
                    prj = prj
                    sa   = sa
                    role = role
                }
            ]
        ]
    ]))
}

resource "google_artifact_registry_repository_iam_member" "gcrio_gar_user" {
  for_each   = { for entry in local.own_repo_access: "${entry.sa}.${entry.role}" => entry }
  project    = data.google_project.project.project_id
  location   = "us"
  repository = "gcr.io"
  role       = each.value.role
  member     = each.value.sa
}

resource "google_artifact_registry_repository_iam_member" "private_gcrio_gar_user" {
  for_each   = { for entry in local.private_repo_access: "${entry.sa}.${entry.prj}.${entry.role}" => entry }
  location   = "us"
  repository = "gcr.io"
  role       = each.value.role
  project    = each.value.prj
  member     = each.value.sa
}
