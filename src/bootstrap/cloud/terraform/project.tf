data "google_project" "project" {
  project_id = var.id
}

resource "google_project_iam_member" "owner_group" {
  count   = var.shared_owner_group == "" ? 0 : 1
  project = data.google_project.project.project_id
  role    = "roles/owner"
  member  = "group:${var.shared_owner_group}"
}

# We can't use google_project_services because Endpoints adds services
# dynamically.

# This is needed to allow creating certificates in GCP.
resource "google_project_service" "certificateauthority" {
  project            = data.google_project.project.project_id
  # Only enable if Google CAS is the Certificate Authority
  count              = var.certificate_provider == "google-cas" ? 1 : 0
  service            = "privateca.googleapis.com"
  disable_on_destroy = false
}

# This is needed for Terraform's data.google_project.project.resource to work.
resource "google_project_service" "cloudbilling" {
  project            = data.google_project.project.project_id
  service            = "cloudbilling.googleapis.com"
  disable_on_destroy = false
}

resource "google_project_service" "cloudiot" {
  project            = data.google_project.project.project_id
  service            = "cloudiot.googleapis.com"
  disable_on_destroy = false
  count              = var.use_cloudiot ? 1 : 0
}

# This is needed for Terraform's data.google_project.project.resource to work.
resource "google_project_service" "cloudresourcemanager" {
  project            = data.google_project.project.project_id
  service            = "cloudresourcemanager.googleapis.com"
  disable_on_destroy = false
}

resource "google_project_service" "compute" {
  project            = data.google_project.project.project_id
  service            = "compute.googleapis.com"
  disable_on_destroy = false
}

resource "google_project_service" "container" {
  project            = data.google_project.project.project_id
  service            = "container.googleapis.com"
  disable_on_destroy = false
}

resource "google_project_service" "containerregistry" {
  project            = data.google_project.project.project_id
  service            = "containerregistry.googleapis.com"
  disable_on_destroy = false
}

resource "google_project_service" "containersecurity" {
  project            = data.google_project.project.project_id
  service            = "containersecurity.googleapis.com"
  disable_on_destroy = false
}

resource "google_project_service" "dns" {
  project            = data.google_project.project.project_id
  service            = "dns.googleapis.com"
  disable_on_destroy = false
}

resource "google_project_service" "endpoints" {
  project            = data.google_project.project.project_id
  service            = "endpoints.googleapis.com"
  disable_on_destroy = false
}

resource "google_project_service" "iam" {
  project            = data.google_project.project.project_id
  service            = "iam.googleapis.com"
  disable_on_destroy = false
}

resource "google_project_service" "iamcredentials" {
  project            = data.google_project.project.project_id
  service            = "iamcredentials.googleapis.com"
  disable_on_destroy = false
}

resource "google_project_service" "logging" {
  project            = data.google_project.project.project_id
  service            = "logging.googleapis.com"
  disable_on_destroy = false
}

resource "google_project_service" "pubsub" {
  project            = data.google_project.project.project_id
  service            = "pubsub.googleapis.com"
  disable_on_destroy = false
}

resource "google_project_service" "servicecontrol" {
  project            = data.google_project.project.project_id
  service            = "servicecontrol.googleapis.com"
  disable_on_destroy = false
}

resource "google_project_service" "servicemanagement" {
  # Needed for Terraform project management through service account
  project            = data.google_project.project.project_id
  service            = "servicemanagement.googleapis.com"
  disable_on_destroy = false
}

resource "google_project_service" "serviceusage" {
  # Needed for Terraform to manage the enabled services.
  project            = data.google_project.project.project_id
  service            = "serviceusage.googleapis.com"
  disable_on_destroy = false
}
