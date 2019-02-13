output "ingress-ip" {
  value = "${google_compute_address.cloud_robotics.address}"
}

output "project-number" {
  value = "${google_project.project.number}"
}
