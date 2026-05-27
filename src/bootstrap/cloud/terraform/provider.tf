provider "google" {
  project = var.id
  region  = var.region
}

data "google_client_config" "default" {}

provider "kubernetes" {
  host                   = "https://${google_container_cluster.cloud-robotics.endpoint}"
  token                  = data.google_client_config.default.access_token
  cluster_ca_certificate = base64decode(google_container_cluster.cloud-robotics.master_auth[0].cluster_ca_certificate)
}

