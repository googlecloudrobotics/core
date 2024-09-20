# Configuration for the following service accounts:
#
# - robot-service@, which is used by workloads on robot clusters to access GCP
#   APIs, as well as services like the k8s-relay though ingress-nginx.
# - human-acl@, which is used as a "virtual permission" for users of the
#   cluster, allowing access to services like Grafana through ingress-nginx.
#   It can also be used to generate tokens for registering new clusters to the
#   cloud.

resource "google_service_account" "robot-service" {
  account_id   = "robot-service"
  display_name = "robot-service"
  project      = data.google_project.project.project_id
  count = var.onprem_federation ? 1 : 0
}

# Allow the the token-vendor to impersonate the "robot-service" service account
# and to create new tokens for it.
data "google_iam_policy" "robot-service" {
  binding {
    # Security note from b/120897889: This permission allows privilege escalation
    # if granted too widely. Make sure that the robot-service can't mint tokens
    # for accounts other than itself! If in doubt, review this section carefully.
    # In particular, this serves as the default service account for all containers
    # running in the GKE cluster
    role = "roles/iam.serviceAccountTokenCreator"

    members = [
      "serviceAccount:${google_service_account.token_vendor.email}",
    ]
  }

  binding {
    role = "roles/iam.serviceAccountUser"

    members = [
      "serviceAccount:${google_service_account.token_vendor.email}",

      # This seemingly nonsensical binding is necessary for the robot auth
      # path in the K8s relay, which has to work with GCP auth tokens.
      "serviceAccount:${google_service_account.robot-service[0].email}",
    ]
  }

  count = var.onprem_federation ? 1 : 0
}

# Bind policy to the "robot-service" service account.
# More: https://cloud.google.com/iam/docs/service-accounts#service_account_permissions
resource "google_service_account_iam_policy" "robot-service" {
  service_account_id = google_service_account.robot-service[0].name
  policy_data        = data.google_iam_policy.robot-service[0].policy_data
  count              = var.onprem_federation ? 1 : 0
}

# TODO(ensonic): check if this still makes sense after GAR migration
resource "google_project_iam_member" "robot-service-account-container-access" {
  project = var.private_image_repositories[count.index]
  count   = var.onprem_federation ? length(var.private_image_repositories) : 0
  role    = "roles/storage.objectViewer"
  member  = "serviceAccount:${google_service_account.robot-service[0].email}"
}

resource "google_project_iam_member" "robot-service-roles" {
  project = data.google_project.project.project_id
  member  = "serviceAccount:${google_service_account.robot-service[0].email}"
  for_each = var.onprem_federation ? toset([
    "roles/cloudtrace.agent",  # Upload cloud traces
    "roles/container.clusterViewer", # Sync CRs from the GKE cluster.
    "roles/logging.logWriter", # Upload text logs to Cloud logging
    # Required to use robot-service@ for GKE clusters that simulate robots
    "roles/monitoring.viewer",
  ]) : toset([])
  role = each.value
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

###
# The following permissions make human-acl@ tokens work with setup_robot.sh.
# To create such tokens, the user needs roles/iam.serviceAccountTokenCreator.
# https://cloud.google.com/iam/docs/create-short-lived-credentials-direct
#
# This also RBAC policy to create Robot CRs, defined in
# src/app_charts/base/cloud/registry-policy.yaml.
###

# Allow reading GCS objects such as setup_robot_crc_version.txt.
resource "google_project_iam_member" "human-acl-object-viewer" {
  project = data.google_project.project.project_id
  role    = "roles/storage.objectViewer"
  member  = "serviceAccount:${google_service_account.human-acl.email}"
}

# Allow robot registration with the token vendor, which checks if the client's
# token can "act as" the human-acl@ SA. We need this binding even if the
# client provided a token for the human-acl@ SA itself.
resource "google_service_account_iam_member" "human-acl-act-as-self" {
  service_account_id = google_service_account.human-acl.name
  role               = "roles/iam.serviceAccountUser"
  member             = "serviceAccount:${google_service_account.human-acl.email}"
}

# Grant permissions to generate tokens for registering new workcell clusters.
# This lets users run:
#   gcloud auth print-access-token \
#     --impersonate-service-account=human-acl@${PROJECT_ID}.iam.gserviceaccount.com
# so they can register new workcell clusters without passing their own tokens
# (which aren't limited to a single GCP project) into the cluster.
resource "google_service_account_iam_member" "human-acl-token-generator" {
  service_account_id = google_service_account.human-acl.name
  role               = "roles/iam.serviceAccountTokenCreator"
  member             = "group:${var.shared_owner_group}"
  count              = var.shared_owner_group == "" ? 0 : 1
}
