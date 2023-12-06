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

variable "additional_regions" {
  description = "Cloud regions to deploy additional relays to"
  type        = map
  default     = {}
}

variable "shared_owner_group" {
  description = "Name of a group to be added as a owner. Leave empty to not use group sharing."
  default     = ""
}

variable "robot_image_reference" {
  description = "Reference to the Docker image installed by the setup-robot script"
}

variable "crc_version" {
  description = "cloudrobotics-core version tag stored with the setup-robot script to align cloud and robot versions."
}

variable "private_image_repositories" {
  description = "Projects with private GCR image repositories where we need to add IAM access rules."
  type        = list
  default     = []
}

variable "certificate_provider" {
  description = "Certificate provider to use to generate certificates for in-cluster services. Should be one of: lets-encrypt, google-cas."
  type        = string
}

variable "certificate_subject_common_name" {
  description = "Certificate Common Name (CN) field"
  type        = string
}

variable "certificate_subject_organization" {
  description = "Certificate Subject Organization (O) field"
  type        = string
}

variable "certificate_subject_organizational_unit" {
  description = "Certificate Subject Organizational Unit (OU) field"
  type        = string
  default     = null
}
