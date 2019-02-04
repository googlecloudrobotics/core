resource "google_compute_address" "cloud_robotics" {
  name       = "cloud-robotics"
  project    = "${google_project.project.name}"
  region     = "${var.region}"
  depends_on = ["google_project_service.compute"]
}
