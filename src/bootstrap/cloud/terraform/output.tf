output "ingress-ip" {
  value = "${google_compute_address.cloud_robotics.address}"
}

