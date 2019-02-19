resource "google_compute_address" "cloud_robotics" {
  name       = "cloud-robotics"
  project    = "${data.google_project.project.project_id}"
  region     = "${var.region}"
  depends_on = ["google_project_service.compute"]
}
