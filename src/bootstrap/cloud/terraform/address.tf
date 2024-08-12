resource "google_compute_address" "cloud_robotics" {
  name       = "cloud-robotics"
  project    = data.google_project.project.project_id
  region     = var.region
  depends_on = [google_project_service.project-services["compute"]]
}

resource "google_compute_address" "cloud_robotics_ar" {
  for_each   = var.additional_regions
  name       = format("%s-%s", each.key, "ar-cloud-robotics")
  project    = data.google_project.project.project_id
  region     = each.value.region
  depends_on = [google_project_service.project-services["compute"]]
}
