# New log bucket with short TTL for access logs
resource "google_logging_project_bucket_config" "remote_access_bucket" {
  project        = data.google_project.project.project_id
  location       = "global"
  retention_days = 2
  bucket_id      = "RemoteAccess"
}

# Log all access logs to this dedicated bucket
resource "google_logging_project_sink" "remote_access_sink" {
  name                   = "remote-access-sink"
  destination            = "logging.googleapis.com/projects/${var.id}/locations/global/buckets/RemoteAccess"
  filter                 = "resource.type=\"k8s_container\" resource.labels.cluster_name=\"cloud-robotics\" (resource.labels.container_name=\"nginx-ingress-controller\" OR resource.labels.container_name=\"oauth2-proxy\")"
  unique_writer_identity = true
}

# Don't store access logs in "_Default" bucket anymore
resource "google_logging_project_exclusion" "remote_access_exclusion" {
  name   = "remote-access-exclusion"
  filter = "resource.type=\"k8s_container\" resource.labels.cluster_name=\"cloud-robotics\" (resource.labels.container_name=\"nginx-ingress-controller\" OR resource.labels.container_name=\"oauth2-proxy\")"
}
