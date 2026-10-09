# IAM Configuration for GKE cluster workloads using Workload Identity.
# See also cluster.tf.

# token-vendor
##############
resource "google_service_account" "token_vendor" {
  account_id   = "token-vendor"
  display_name = "token-vendor"
  project      = data.google_project.project.project_id
}

# Allow the app-token-vendor/token-vendor Kubernetes SA to use this GCP SA.
resource "google_service_account_iam_policy" "token_vendor" {
  service_account_id = google_service_account.token_vendor.name
  policy_data        = data.google_iam_policy.token_vendor.policy_data
  # Avoid Error 400: Identity Pool does not exist (my-project.svc.id.goog).
  depends_on = [google_container_cluster.cloud-robotics]
}

data "google_iam_policy" "token_vendor" {
  binding {
    role = "roles/iam.workloadIdentityUser"
    members = [
      "serviceAccount:${data.google_project.project.project_id}.svc.id.goog[app-token-vendor/token-vendor]"
    ]
  }
}

# Note: the policy in service-account.tf allows the token-vendor to create
# new tokens for the robot-service@ service account.

# cert-manager
##############

resource "google_service_account" "cert_manager" {
  account_id   = "cert-manager"
  display_name = "cert-manager"
  project      = data.google_project.project.project_id
}

data "google_iam_policy" "cert_manager" {
  binding {
    role = "roles/iam.workloadIdentityUser"
    members = [
      "serviceAccount:${data.google_project.project.project_id}.svc.id.goog[default/cert-manager]",
    ]
  }
}

resource "google_service_account_iam_policy" "cert_manager" {
  service_account_id = google_service_account.cert_manager.id
  policy_data        = data.google_iam_policy.cert_manager.policy_data
  depends_on         = [google_container_cluster.cloud-robotics]
}

# Instead of granting dns.admin on the whole project as recommended, grant
# reader on the project (needed to list managed zones) and admin on the
# individual zones that cert-manager uses.
resource "google_project_iam_member" "cert_manager_dns_reader" {
  project = data.google_project.project.project_id
  role    = "roles/dns.reader"
  member  = google_service_account.cert_manager.member
}

# grafana
##############

resource "google_service_account" "grafana" {
  account_id   = "grafana"
  display_name = "grafana"
  project      = data.google_project.project.project_id
}

data "google_iam_policy" "grafana" {
  binding {
    role = "roles/iam.workloadIdentityUser"
    members = [
      "serviceAccount:${data.google_project.project.project_id}.svc.id.goog[app-grafana/grafana]",
    ]
  }
}

resource "google_service_account_iam_policy" "grafana" {
  service_account_id = google_service_account.grafana.id
  policy_data        = data.google_iam_policy.grafana.policy_data
  depends_on         = [google_container_cluster.cloud-robotics]
}

resource "google_project_iam_member" "grafana_roles" {
  project = data.google_project.project.project_id
  role    = "roles/cloudtrace.user"
  member  = google_service_account.grafana.member
}

# crc-relay-server
##################

resource "google_service_account" "crc_relay_server" {
  account_id   = "crc-relay-server"
  display_name = "crc-relay-server"
  project      = data.google_project.project.project_id
}

data "google_iam_policy" "crc_relay_server" {
  binding {
    role = "roles/iam.workloadIdentityUser"
    members = [
      "serviceAccount:${data.google_project.project.project_id}.svc.id.goog[app-prometheus/prometheus-relay-server]",
      "serviceAccount:${data.google_project.project.project_id}.svc.id.goog[app-k8s-relay/kubernetes-relay-server]",
    ]
  }
}

resource "google_service_account_iam_policy" "crc_relay_server" {
  service_account_id = google_service_account.crc_relay_server.id
  policy_data        = data.google_iam_policy.crc_relay_server.policy_data
  depends_on         = [google_container_cluster.cloud-robotics]
}

resource "google_project_iam_member" "crc_relay_server_roles" {
  project = data.google_project.project.project_id
  role    = "roles/cloudtrace.agent"
  member  = google_service_account.crc_relay_server.member
}

# chart-assignment-controller
#############################

resource "google_service_account" "chart_assignment_controller" {
  account_id   = "chart-assignment-controller"
  display_name = "chart-assignment-controller"
  project      = data.google_project.project.project_id
}

data "google_iam_policy" "chart_assignment_controller" {
  binding {
    role = "roles/iam.workloadIdentityUser"
    members = [
      "serviceAccount:${data.google_project.project.project_id}.svc.id.goog[default/chart-assignment-controller]",
    ]
  }
}

resource "google_service_account_iam_policy" "chart_assignment_controller" {
  service_account_id = google_service_account.chart_assignment_controller.id
  policy_data        = data.google_iam_policy.chart_assignment_controller.policy_data
  depends_on         = [google_container_cluster.cloud-robotics]
}

resource "google_project_iam_member" "chart_assignment_controller_roles" {
  project = data.google_project.project.project_id
  role    = "roles/cloudtrace.agent"
  member  = google_service_account.chart_assignment_controller.member
}


