# IAM Configuration for GKE cluster workloads using Workload Identity.
# See also cluster.tf.

# token-vendor
##############
resource "google_service_account" "token_vendor" {
  account_id   = "token-vendor"
  display_name = "token-vendor"
  project      = data.google_project.project.project_id
}

# Allow the default/token-vendor Kubernetes SA to use this GCP SA.
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
      "serviceAccount:${data.google_project.project.project_id}.svc.id.goog[default/token-vendor]"
    ]
  }
}

# Allow the token-vendor to push robot public keys to the Cloud IoT Device
# Registry.
resource "google_project_iam_member" "token_vendor_cloudiot_provisioner" {
  project = data.google_project.project.project_id
  role    = "roles/cloudiot.provisioner"
  member  = "serviceAccount:${google_service_account.token_vendor.email}"
}

# Note: the policy in service-account.tf allows the token-vendor to create
# new tokens for the robot-service@ service account.
