resource "google_storage_bucket" "robot" {
  name                        = "${var.id}-robot"
  location                    = var.region
  storage_class               = "REGIONAL"
  uniform_bucket_level_access = "true"
  force_destroy               = "true"
  depends_on                  = [google_project_service.compute]
}

resource "google_storage_bucket_object" "setup_robot_image_reference" {
  name          = "setup_robot_image_reference.txt"
  content       = var.robot_image_reference
  bucket        = google_storage_bucket.robot.name
  cache_control = "private, max-age=0, no-transform"
}

resource "google_storage_bucket_object" "setup_robot_crc_version" {
  name          = "setup_robot_crc_version.txt"
  content       = var.crc_version
  bucket        = google_storage_bucket.robot.name
  cache_control = "private, max-age=0, no-transform"
}

resource "google_storage_bucket_object" "setup_robot" {
  name          = "setup_robot.sh"
  source        = "../../robot/setup_robot.sh"
  bucket        = google_storage_bucket.robot.name
  cache_control = "private, max-age=0, no-transform"
}
