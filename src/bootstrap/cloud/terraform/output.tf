output "ingress-ip" {
  value = google_compute_address.cloud_robotics.address
}

output "ingress-ip-ar" {
  value = {
    for address in google_compute_address.cloud_robotics_ar : address.name => address.address
  }
}

output "cluster-location" {
  value = google_container_cluster.cloud-robotics.location
}

output "service-accounts" {
  value = {
    human         = google_service_account.human-acl
    robot-service = google_service_account.robot-service
  }
}
