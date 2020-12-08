resource "google_dns_managed_zone" "external-dns" {
  name        = "external-dns"
  dns_name    = "${var.domain}."
  count       = var.domain == "" ? 0 : 1
  description = "Automatically managed zone by kubernetes.io/external-dns"
  depends_on  = [google_project_service.dns]
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
