resource "google_service_account" "robot-service" {
  account_id   = "robot-service"
  display_name = "robot-service"
  project      = data.google_project.project.project_id
}

# Allow the the standard compute service account to impersonate the "robot-service"
# service account and to create new tokens for it.
data "google_iam_policy" "robot-service" {
  binding {
    # Security note from b/120897889: This permission allows privilege escalation
    # if granted too widely. Make sure that the robot-service can't mint tokens
    # for accounts other than itself! If in doubt, review this section carefully.
    # In particular, this serves as the default service account for all containers
    # running in the GKE cluster
    role = "roles/iam.serviceAccountTokenCreator"

    members = [
      "serviceAccount:${data.google_project.project.number}-compute@developer.gserviceaccount.com",
    ]
  }

  binding {
    role = "roles/iam.serviceAccountUser"

    members = [
      "serviceAccount:${data.google_project.project.number}-compute@developer.gserviceaccount.com",

      # This seemingly nonsensical binding is necessary for the robot auth
      # path in the K8s relay, which has to work with GCP auth tokens.
      "serviceAccount:${google_service_account.robot-service.email}",
    ]
  }
}

# Bind policy to the "robot-service" service account.
# More: https://cloud.google.com/iam/docs/service-accounts#service_account_permissions
resource "google_service_account_iam_policy" "robot-service" {
  service_account_id = google_service_account.robot-service.name
  policy_data        = data.google_iam_policy.robot-service.policy_data
}

resource "google_project_iam_member" "robot-service-storage" {
  project = data.google_project.project.project_id
  role    = "roles/storage.objectAdmin"
  member  = "serviceAccount:${google_service_account.robot-service.email}"
}

resource "google_project_iam_member" "robot-service-account-container-access" {
  project = var.private_image_repositories[count.index]
  count   = length(var.private_image_repositories)
  role    = "roles/storage.objectViewer"
  member  = "serviceAccount:${google_service_account.robot-service.email}"
}

resource "google_project_iam_member" "robot-service-datastore" {
  project = data.google_project.project.project_id
  role    = "roles/datastore.user"
  member  = "serviceAccount:${google_service_account.robot-service.email}"
}

resource "google_project_iam_member" "robot-service-monitoring" {
  project = data.google_project.project.project_id
  role    = "roles/monitoring.metricWriter"
  member  = "serviceAccount:${google_service_account.robot-service.email}"
}

resource "google_project_iam_member" "robot-service-tracing" {
  project = data.google_project.project.project_id
  role    = "roles/cloudtrace.agent"
  member  = "serviceAccount:${google_service_account.robot-service.email}"
}

resource "google_project_iam_member" "robot-service-logging" {
  project = data.google_project.project.project_id
  role    = "roles/logging.logWriter"
  member  = "serviceAccount:${google_service_account.robot-service.email}"
}

resource "google_project_iam_member" "robot-service-kubernetes" {
  project = data.google_project.project.project_id

  # TODO(rodrigoq): migrate to RBAC and remove roles/container.developer
  role = var.cr_syncer_rbac == "true" ? "roles/container.clusterViewer" : "roles/container.developer"

  member = "serviceAccount:${google_service_account.robot-service.email}"
}

# The roles above are sufficient for "real robots". The following roles let us
# use robot-service@ for GKE clusters that simulate robots.
resource "google_project_iam_member" "robot_service_stackdriver_writer" {
  project = data.google_project.project.project_id
  role    = "roles/stackdriver.resourceMetadata.writer"
  member  = "serviceAccount:${google_service_account.robot-service.email}"
}

resource "google_project_iam_member" "robot_service_monitoring_viewer" {
  project = data.google_project.project.project_id
  role    = "roles/monitoring.viewer"
  member  = "serviceAccount:${google_service_account.robot-service.email}"
}

# The name is slightly misleading - this is about the compute service account.
# However, renaming in Terraform is hard :-(.
resource "google_project_iam_member" "robot-service-container-access" {
  project    = var.private_image_repositories[count.index]
  count      = length(var.private_image_repositories)
  role       = "roles/storage.objectViewer"
  member     = "serviceAccount:${data.google_project.project.number}-compute@developer.gserviceaccount.com"
  depends_on = [google_project_service.compute]
}

resource "google_service_account" "human-acl" {
  account_id   = "human-acl"
  display_name = "human-acl"
  project      = data.google_project.project.project_id
}

resource "google_service_account_iam_member" "human-acl-shared-owner-account-user" {
  count              = var.shared_owner_group == "" ? 0 : 1
  service_account_id = google_service_account.human-acl.name
  role               = "roles/iam.serviceAccountUser"
  member             = "group:${var.shared_owner_group}"
}
