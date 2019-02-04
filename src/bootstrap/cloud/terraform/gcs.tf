resource "google_storage_bucket" "robot" {
  name          = "${var.id}-robot"
  location      = "${var.region}"
  storage_class = "REGIONAL"
  force_destroy = "true"
  depends_on    = ["google_project_service.compute"]
}

resource "google_storage_bucket" "tools" {
  name          = "${var.id}-tools"
  location      = "${var.region}"
  storage_class = "REGIONAL"
  force_destroy = "true"
  depends_on    = ["google_project_service.compute"]
}
