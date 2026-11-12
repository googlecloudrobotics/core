# Projects with a custom domain are served on that domain only, so they don't
# get the www.endpoints name. Changing var.domain between "" and a value
# deletes/creates this service: a deleted service is soft-deleted for 30 days
# and must be undeleted manually (`gcloud endpoints services undelete`) to be
# used again within that time.
resource "google_endpoints_service" "www_service" {
  count        = var.domain == "" ? 1 : 0
  service_name = "www.endpoints.${var.id}.cloud.goog"
  project      = var.id
  openapi_config = templatefile(
    "${path.module}/www.yaml",
    {
      GCP_PROJECT_ID = var.id
      INGRESS_IP     = google_compute_address.cloud_robotics.address
    }
  )
}

# Keeps the existing service of projects without a custom domain (no-op plan).
moved {
  from = google_endpoints_service.www_service
  to   = google_endpoints_service.www_service[0]
}
