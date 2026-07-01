resource "google_storage_bucket" "robot" {
  name                        = "${data.google_project.project.project_id}-robot"
  location                    = var.region
  storage_class               = "REGIONAL"
  uniform_bucket_level_access = "true"
  force_destroy               = "true"
  depends_on                  = [google_project_service.project-services["storage-component.googleapis.com"]]
  count                       = var.onprem_federation ? 1 : 0
}

resource "google_storage_bucket_object" "setup_robot_image_reference" {
  name          = "setup_robot_image_reference.txt"
  content       = var.robot_image_reference
  bucket        = google_storage_bucket.robot[0].name
  cache_control = "private, max-age=0, no-transform"
  count         = var.onprem_federation ? 1 : 0
}

resource "google_storage_bucket_object" "setup_robot_crc_version" {
  name          = "setup_robot_crc_version.txt"
  content       = var.crc_version
  bucket        = google_storage_bucket.robot[0].name
  cache_control = "private, max-age=0, no-transform"
  count         = var.onprem_federation ? 1 : 0
}

resource "google_storage_bucket_object" "setup_robot" {
  name          = "setup_robot.sh"
  source        = "${path.module}/../../robot/setup_robot.sh"
  bucket        = google_storage_bucket.robot[0].name
  cache_control = "private, max-age=0, no-transform"
  count         = var.onprem_federation ? 1 : 0
}

resource "google_storage_bucket" "config_store" {
  name                        = "${data.google_project.project.project_id}-cloud-robotics-config"
  location                    = "US"
  storage_class               = "STANDARD"
  force_destroy               = true
  uniform_bucket_level_access = true
}

resource "google_storage_bucket_object" "config_store_crc_version" {
  name          = "crc_version.txt"
  content       = var.crc_version
  bucket        = google_storage_bucket.config_store.name
  cache_control = "private, max-age=0, no-transform"
}

resource "random_id" "cloud_robotics_cookie_secret" {
  byte_length = 16

  count = var.cookie_secret == null ? 1 : 0
}

resource "google_storage_bucket_object" "crc_config" {
  name = "config.sh"
  content = <<EOF
#!/usr/bin/env bash
GCP_PROJECT_ID='${data.google_project.project.project_id}'
GCP_REGION='${var.region}'
GCP_ZONE='${var.zone}'
CLOUD_ROBOTICS_CONTAINER_REGISTRY='gcr.io/${data.google_project.project.project_id}'
PRIVATE_DOCKER_PROJECTS='${join(" ", var.private_image_repositories)}'
CLOUD_ROBOTICS_SHARED_OWNER_GROUP='${var.shared_owner_group}'
CLOUD_ROBOTICS_COOKIE_SECRET='${var.cookie_secret != null ? var.cookie_secret : random_id.cloud_robotics_cookie_secret[0].b64_url}'
CLOUD_ROBOTICS_OAUTH2_CLIENT_ID='${var.oauth2_client != null ? var.oauth2_client.client_id : ""}'
CLOUD_ROBOTICS_OAUTH2_CLIENT_SECRET='${var.oauth2_client != null ? var.oauth2_client.secret : ""}'
CLOUD_ROBOTICS_DOMAIN='${var.domain}'
GCP_NODE_VM_TYPE='${var.node_machine_type}'
GKE_MIN_NODES='${var.min_node_count}'
GKE_MAX_NODES='${var.max_node_count}'
CLOUD_ROBOTICS_INGRESS_IP='${google_compute_address.cloud_robotics.address}'
declare -A CLOUD_ROBOTICS_INGRESS_IP_AR
${join("\n", [
  for address in google_compute_address.cloud_robotics_ar : "CLOUD_ROBOTICS_INGRESS_IP_AR[\"${address.name}\"]=\"${address.address}\""
])}
APP_MANAGEMENT=true
ONPREM_FEDERATION="${var.onprem_federation}"
GKE_SECRET_MANAGER_PLUGIN='${var.secret_manager_plugin}'
EOF
  bucket        = google_storage_bucket.config_store.name
  cache_control = "private, max-age=0, no-transform"
}
