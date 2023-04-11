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

# Allow the token-vendor to push robot public keys to the Cloud IoT Device
# Registry.
resource "google_project_iam_member" "token_vendor_cloudiot_provisioner" {
  project = data.google_project.project.project_id
  role    = "roles/cloudiot.provisioner"
  member  = "serviceAccount:${google_service_account.token_vendor.email}"
  count = var.use_cloudiot ? 1 : 0
}

# Note: the policy in service-account.tf allows the token-vendor to create
# new tokens for the robot-service@ service account.

# cert-manager-google-cas-issuer
################################

###
# The following resources enable Google's Certificate Authority Service (CAS) support.
# They are required to access the CAS service to generate certificates for cluster resources.
###

resource "google_service_account" "google-cas-issuer" {
  count        = var.certificate_provider == "google-cas" ? 1 : 0
  account_id   = "sa-google-cas-issuer"
  display_name = "sa-google-cas-issuer"
  description  = "Service account used by GKE cert-manager's google-cas-issuer to emit certificates using Google's Certificate Authority Service (CAS)."
}

# Bind IAM policies to the "sa-google-cas-issuer" service account.

# Allow the SA to create private CA pool certificates.
resource "google_privateca_ca_pool_iam_member" "ca-pool-workload-identity" {
  count   = var.certificate_provider == "google-cas" ? 1 : 0

  ca_pool = google_privateca_ca_pool.ca_pool[0].id
  role = "roles/privateca.certificateManager"
  member = "serviceAccount:${google_service_account.google-cas-issuer[0].email}"
}

# Define IAM policy for the workload identity user.
# This allows the Kubernetes service account to act as the GKE service account.
data "google_iam_policy" "google-cas-issuer" {
  count   = var.certificate_provider == "google-cas" ? 1 : 0

  binding {
    role = "roles/iam.workloadIdentityUser"
    members = [
      "serviceAccount:${data.google_project.project.project_id}.svc.id.goog[default/cert-manager-google-cas-issuer]",
    ]
  }
}

resource "google_service_account_iam_policy" "google-cas-issuer" {
  count              = var.certificate_provider == "google-cas" ? 1 : 0
  service_account_id = google_service_account.google-cas-issuer[0].id
  policy_data        = data.google_iam_policy.google-cas-issuer[0].policy_data
  depends_on         = [google_container_cluster.cloud-robotics]
}
