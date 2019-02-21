resource "google_storage_bucket" "robot" {
  name          = "${var.id}-robot"
  location      = "${var.region}"
  storage_class = "REGIONAL"
  force_destroy = "true"
  depends_on    = ["google_project_service.compute"]
}

resource "google_storage_bucket_object" "setup_robot_image_reference" {
  name          = "setup_robot_image_reference.txt"
  source        = "../../../../bazel-genfiles/src/bootstrap/robot/setup_robot_image_reference.txt"
  bucket        = "${google_storage_bucket.robot.name}"
  cache_control = "private, max-age=0, no-transform"
}

resource "google_storage_object_access_control" "setup_robot_image_reference_ac" {
  object = "${google_storage_bucket_object.setup_robot_image_reference.output_name}"
  bucket = "${google_storage_bucket.robot.name}"
  role   = "READER"
  entity = "allUsers"
}

resource "google_storage_bucket_object" "setup_robot" {
  name          = "setup_robot.sh"
  source        = "../../robot/setup_robot.sh"
  bucket        = "${google_storage_bucket.robot.name}"
  cache_control = "private, max-age=0, no-transform"
}

resource "google_storage_object_access_control" "setup_robot_ac" {
  object = "${google_storage_bucket_object.setup_robot.output_name}"
  bucket = "${google_storage_bucket.robot.name}"
  role   = "READER"
  entity = "allUsers"
}

resource "google_storage_bucket_object" "install_k8s_on_robot" {
  name          = "install_k8s_on_robot.sh"
  source        = "../../robot/install_k8s_on_robot.sh"
  bucket        = "${google_storage_bucket.robot.name}"
  cache_control = "private, max-age=0, no-transform"
}

resource "google_storage_object_access_control" "install_k8s_on_robot_ac" {
  object = "${google_storage_bucket_object.install_k8s_on_robot.output_name}"
  bucket = "${google_storage_bucket.robot.name}"
  role   = "READER"
  entity = "allUsers"
}

resource "google_storage_bucket" "tools" {
  name          = "${var.id}-tools"
  location      = "${var.region}"
  storage_class = "REGIONAL"
  force_destroy = "true"
  depends_on    = ["google_project_service.compute"]
}
