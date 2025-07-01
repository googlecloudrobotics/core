# Do not use count to create these 2 conditionally. A deleted service needs to be manually undeleted
# within 30 days of deletion if one wishes to use it again.
resource "google_endpoints_service" "www_service" {
  service_name = "www.endpoints.${var.id}.cloud.goog"
  project      = var.id
  openapi_config = templatefile(
    "www.yaml",
    {
      GCP_PROJECT_ID = var.id
      INGRESS_IP     = google_compute_address.cloud_robotics.address
    }
  )
}
