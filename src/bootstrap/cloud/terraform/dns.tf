resource "google_dns_managed_zone" "external-dns" {
  name        = "external-dns"
  dns_name    = "${var.domain}."
  count       = var.domain == "" ? 0 : 1
  # This is used to be true but is no longer, but we keep it here so that
  # Terraform doesn't delete and recreate the zone.
  description = "Automatically managed zone by kubernetes.io/external-dns"
  depends_on  = [google_project_service.project-services["dns.googleapis.com"]]
}

resource "google_dns_record_set" "www-entry" {
  name  = "${var.domain}."
  count = var.domain == "" ? 0 : 1
  type  = "A"

  ttl = 300

  rrdatas      = [google_compute_address.cloud_robotics.address]
  managed_zone = google_dns_managed_zone.external-dns[0].name
  project      = google_dns_managed_zone.external-dns[0].project
}


# This is a record, for example, for europe-west1.subdomain.example.com.
# It's used to serve requests in a region closer to the user.
resource "google_dns_record_set" "www-entry-ar" {
  for_each = var.domain == "" ? {} : var.additional_regions

  name = "${each.key}.${var.domain}."
  type = "A"

  ttl = 300

  rrdatas      = [google_compute_address.cloud_robotics_ar[each.key].address]
  managed_zone = google_dns_managed_zone.external-dns[0].name
  project      = google_dns_managed_zone.external-dns[0].project
}

# Allow cert-manager to solve DNS01 challenges in this zone.
data "google_iam_policy" "external-dns" {
  binding {
    role = "roles/dns.admin"
    members = [
      "serviceAccount:${google_service_account.cert_manager.email}"
    ]
  }
}

resource "google_dns_managed_zone_iam_policy" "external-dns" {
  count       = var.domain == "" ? 0 : 1
  project = google_dns_managed_zone.external-dns[0].project
  managed_zone = google_dns_managed_zone.external-dns[0].name
  policy_data = data.google_iam_policy.external-dns.policy_data
}
