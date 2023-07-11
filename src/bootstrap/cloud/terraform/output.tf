output "ingress-ip" {
  value = google_compute_address.cloud_robotics.address
}

output "ingress-ip-ar" {
  value = {
    for address in google_compute_address.cloud_robotics_ar : address.name => address.address
  }
}
