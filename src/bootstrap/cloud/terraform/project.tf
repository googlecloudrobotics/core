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
resource "google_project_service" "project-services" {
  project            = data.google_project.project.project_id
  disable_on_destroy = false
  for_each = toset(concat([
    "artifactregistry.googleapis.com",
    # Next 2 are needed for Terraform's data.google_project.project.resource to work.
    "cloudbilling.googleapis.com",
    "cloudresourcemanager.googleapis.com",
    "compute.googleapis.com",
    "container.googleapis.com",
    "containersecurity.googleapis.com",
    "dns.googleapis.com",
    "endpoints.googleapis.com",
    "iam.googleapis.com",
    "iamcredentials.googleapis.com",
    "logging.googleapis.com",
    "servicecontrol.googleapis.com",
    # Next 2 are needed fro terraform again
    "servicemanagement.googleapis.com",
    "serviceusage.googleapis.com",
    "storage-component.googleapis.com",
    ], length(var.additional_regions) == 0 ? [] : [
    # Following APIs are only needed when using multi-cluster gateways.
    "gkeconnect.googleapis.com",
    "gkehub.googleapis.com",
    "trafficdirector.googleapis.com",
    "multiclusterservicediscovery.googleapis.com",
    "multiclusteringress.googleapis.com",
  ]))
  service = each.value
}

# This is needed to allow creating certificates in GCP.
resource "google_project_service" "certificateauthority" {
  project = data.google_project.project.project_id
  # Only enable if Google CAS is the Certificate Authority
  count              = var.certificate_provider == "google-cas" ? 1 : 0
  service            = "privateca.googleapis.com"
  disable_on_destroy = false
}
