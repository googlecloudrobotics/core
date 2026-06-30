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

  count = var.provisioned_by_deploy_script ? 0 : 1
}

resource "google_storage_bucket_object" "crc_config" {
  name = "config.sh"
  content = templatefile("${path.module}/config.sh.tmpl", {
    project_id                   = data.google_project.project.project_id
    region                       = var.region
    zone                         = var.zone
    private_image_repositories   = var.private_image_repositories
    shared_owner_group           = var.shared_owner_group
    domain                       = var.domain
    cloud_robotics_cookie_secret = random_id.cloud_robotics_cookie_secret[0].b64_url
    oauth2_client_id             = var.oauth2_client != null ? var.oauth2_client.client_id : ""
    oauth2_client_secret         = var.oauth2_client != null ? var.oauth2_client.secret : ""
    onprem_federation            = var.onprem_federation
    secret_manager_plugin        = var.secret_manager_plugin
    node_machine_type            = var.node_machine_type
    min_node_count               = var.min_node_count
    max_node_count               = var.max_node_count
    ingress_ip                   = google_compute_address.cloud_robotics.address
    ingress_ip_ar_bash = join("\n", [
      for address in google_compute_address.cloud_robotics_ar : "CLOUD_ROBOTICS_INGRESS_IP_AR[\"${address.name}\"]=\"${address.address}\""
    ])
  })
  bucket        = google_storage_bucket.config_store.name
  cache_control = "private, max-age=0, no-transform"

  count = var.provisioned_by_deploy_script ? 0 : 1
}
