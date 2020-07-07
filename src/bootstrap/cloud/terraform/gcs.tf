resource "google_storage_bucket" "robot" {
  name               = "${var.id}-robot"
  location           = "${var.region}"
  storage_class      = "REGIONAL"
  bucket_policy_only = "true"
  force_destroy      = "true"
  depends_on         = ["google_project_service.compute"]
}

resource "google_storage_bucket_object" "setup_robot_image_reference" {
  name          = "setup_robot_image_reference.txt"
  content       = "${var.robot_image_reference}"
  bucket        = "${google_storage_bucket.robot.name}"
  cache_control = "private, max-age=0, no-transform"
}

resource "google_storage_bucket_object" "setup_robot" {
  name          = "setup_robot.sh"
  source        = "../../robot/setup_robot.sh"
  bucket        = "${google_storage_bucket.robot.name}"
  cache_control = "private, max-age=0, no-transform"
}

resource "google_storage_bucket_object" "install_k8s_on_robot" {
  name          = "install_k8s_on_robot.sh"
  source        = "../../robot/install_k8s_on_robot.sh"
  bucket        = "${google_storage_bucket.robot.name}"
  cache_control = "private, max-age=0, no-transform"
}

resource "google_storage_bucket" "tools" {
  name               = "${var.id}-tools"
  location           = "${var.region}"
  storage_class      = "REGIONAL"
  bucket_policy_only = "true"
  force_destroy      = "true"
  depends_on         = ["google_project_service.compute"]
}
