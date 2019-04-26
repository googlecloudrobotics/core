variable "name" {
  description = "Deployment's human-readable name (my-project)"
}

variable "id" {
  description = "Deployment's project id (my-project)"
}

variable "domain" {
  description = "Deployment domain (www.example.com)"
}

variable "zone" {
  description = "Cloud zone to deploy to (europe-west1-c)"
}

variable "region" {
  description = "Cloud region to deploy to (europe-west1)"
}

variable "shared_owner_group" {
  description = "Name of a group to be added as a owner. Leave empty to not use group sharing."
  default     = ""
}

variable "robot_image_reference" {
  description = "Reference to the Docker image installed by the  setup-robot script"
}

variable "private_image_repositories" {
  description = "Projects with private GCR image repositories where we need to add IAM access rules."
  type        = "list"
  default     = []
}
