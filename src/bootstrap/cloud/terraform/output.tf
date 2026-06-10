output "ingress-ip" {
  value = var.custom_ingress_ip != "" ? var.custom_ingress_ip : google_compute_address.cloud_robotics.address
}

output "ingress-ip-ar" {
  value = {
    for address in google_compute_address.cloud_robotics_ar : address.name => address.address
  }
}

output "cluster-location" {
  value = google_container_cluster.cloud-robotics.location
}
