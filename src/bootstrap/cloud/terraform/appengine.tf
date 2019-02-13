# Only used for datastore
resource "google_app_engine_application" "app" {
  project     = "${data.google_project.project.project_id}"
  location_id = "europe-west"
}
