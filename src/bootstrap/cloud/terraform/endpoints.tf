resource "google_endpoints_service" "map" {
  service_name = "map.endpoints.${var.id}.cloud.goog"
  project      = data.google_project.project.project_id
  grpc_config = templatefile(
    "../../../../src/proto/map/map.yaml",
    {
      GCP_PROJECT_ID = var.id
    }
  )
  protoc_output_base64 = filebase64("../../../../bazel-bin/src/proto/map/proto_descriptor.pb")
}

# Do not use count to create these 2 conditionally. A deleted sevice needs to be manually undeleted
# with 30 days of deletion if one wishes to use it again.
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
