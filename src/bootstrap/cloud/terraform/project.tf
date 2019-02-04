resource "google_project" "project" {
  project_id      = "${var.id}"
  name            = "${var.name}"
  billing_account = "${var.billing_account}"
  folder_id       = "${var.project_folder_id}"

  lifecycle {
    # Never destroy projects, because they take weeks to recreate.
    prevent_destroy = true
  }
}

resource "google_project_iam_member" "owner_group" {
  count   = "${var.shared_owner_group == "" ? 0 : 1}"
  project = "${google_project.project.project_id}"
  role    = "roles/owner"
  member  = "group:${var.shared_owner_group}"
}

# We can't use google_project_services because Endpoints adds services
# dynamically.

# This is needed for Terraform's google_project.project resource to work.
resource "google_project_service" "cloudbilling" {
  project = "${google_project.project.project_id}"
  service = "cloudbilling.googleapis.com"
}

resource "google_project_service" "cloudiot" {
  project = "${google_project.project.project_id}"
  service = "cloudiot.googleapis.com"
}

# This is needed for Terraform's google_project.project resource to work.
resource "google_project_service" "cloudresourcemanager" {
  project = "${google_project.project.project_id}"
  service = "cloudresourcemanager.googleapis.com"
}

resource "google_project_service" "datastore" {
  project = "${google_project.project.project_id}"
  service = "datastore.googleapis.com"
}

resource "google_project_service" "compute" {
  project = "${google_project.project.project_id}"
  service = "compute.googleapis.com"
}

resource "google_project_service" "container" {
  project = "${google_project.project.project_id}"
  service = "container.googleapis.com"
}

resource "google_project_service" "containerregistry" {
  project = "${google_project.project.project_id}"
  service = "containerregistry.googleapis.com"
}

resource "google_project_service" "dns" {
  project = "${google_project.project.project_id}"
  service = "dns.googleapis.com"
}

resource "google_project_service" "endpoints" {
  project = "${google_project.project.project_id}"
  service = "endpoints.googleapis.com"
}

resource "google_project_service" "iam" {
  project = "${google_project.project.project_id}"
  service = "iam.googleapis.com"
}

resource "google_project_service" "iamcredentials" {
  project = "${google_project.project.project_id}"
  service = "iamcredentials.googleapis.com"
}

resource "google_project_service" "logging" {
  project = "${google_project.project.project_id}"
  service = "logging.googleapis.com"
}

resource "google_project_service" "pubsub" {
  project = "${google_project.project.project_id}"
  service = "pubsub.googleapis.com"
}

resource "google_project_service" "servicecontrol" {
  project = "${google_project.project.project_id}"
  service = "servicecontrol.googleapis.com"
}

resource "google_project_service" "servicemanagement" {
  # Needed for Terraform project management through service account
  project = "${google_project.project.project_id}"
  service = "servicemanagement.googleapis.com"
}

resource "google_project_service" "serviceusage" {
  # Needed for Terraform to manage the enabled services.
  project = "${google_project.project.project_id}"
  service = "serviceusage.googleapis.com"
}
