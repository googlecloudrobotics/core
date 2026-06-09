output "ingress-ip" {
  value = var.use_alb ? google_compute_global_address.alb[0].address : google_compute_address.cloud_robotics.address
}

output "alb-ip" {
  value = var.use_alb ? google_compute_global_address.alb[0].address : null
}

output "ingress-ip-ar" {
  value = {
    for address in google_compute_address.cloud_robotics_ar : address.name => address.address
  }
}

output "cluster-location" {
  value = google_container_cluster.cloud-robotics.location
}
