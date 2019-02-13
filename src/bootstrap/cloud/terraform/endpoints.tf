data "template_file" "map_yaml" {
  template = "${file("../../../../src/proto/map/map.yaml")}"

  vars {
    GCP_PROJECT_ID = "${var.id}"
  }
}

resource "google_endpoints_service" "map" {
  service_name         = "map.endpoints.${var.id}.cloud.goog"
  project              = "${data.google_project.project.name}"
  grpc_config          = "${data.template_file.map_yaml.rendered}"
  protoc_output_base64 = "${base64encode(file("../../../../bazel-bin/src/proto/map/proto_descriptor.pb"))}"
}

# Do not use count to create these 2 conditionally. A deleted sevice needs to be manually undeleted
# with 30 days of deletion if one wishes to use it again.
data "template_file" "www_yaml" {
  template = "${file("www.yaml")}"

  vars {
    GCP_PROJECT_ID = "${var.id}"
    INGRESS_IP     = "${google_compute_address.cloud_robotics.address}"
  }
}

resource "google_endpoints_service" "www_service" {
  service_name   = "www.endpoints.${var.id}.cloud.goog"
  project        = "${var.id}"
  openapi_config = "${data.template_file.www_yaml.rendered}"
}
