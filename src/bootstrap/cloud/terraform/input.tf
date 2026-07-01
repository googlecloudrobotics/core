variable "name" {
  description = "Deployment's human-readable name (my-project)"
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
  type        = map(any)
  default     = {}
}

variable "shared_owner_group" {
  description = "Name of a group to be added as a owner. Leave empty to not use group sharing."
}

variable "robot_image_reference" {
  description = "Reference to the Docker image installed by the setup-robot script"
}

variable "crc_version" {
  description = "cloudrobotics-core version tag stored with the setup-robot script to align cloud and robot versions."
}

variable "private_image_repositories" {
  description = "Projects with private GCR image repositories where we need to add IAM access rules."
  type        = list(any)
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

variable "cluster_type" {
  description = "GKE cluster type. Must be one of {zonal,regional}."
  type        = string

  validation {
    condition     = contains(["zonal", "regional"], var.cluster_type)
    error_message = "Must be either \"zonal\" or \"regional\"."
  }
}

variable "datapath_provider" {
  description = "Whether to use Dataplane v1 or v2 (DATAPATH_PROVIDER_UNSPECIFIED or ADVANCED_DATAPATH)."
  type        = string
  validation {
    condition = contains(["DATAPATH_PROVIDER_UNSPECIFIED", "ADVANCED_DATAPATH"], var.datapath_provider)
    error_message = "Must be either \"DATAPATH_PROVIDER_UNSPECIFIED\" or \"ADVANCED_DATAPATH\"."
  }
}

variable "onprem_federation" {
  description = "Enable google cloud robotics layer 1"
  type        = bool
}

variable "secret_manager_plugin" {
  description = "Enable GKE secret manager integration with GKE"
  type        = bool
}

variable "node_machine_type" {
  description = "GCP VM type for GKE nodes"
  type        = string
}

variable "node_disk_type" {
  description = "Disk type for GKE nodes. If null, it will be automatically chosen based on machine type."
  type        = string
}

variable "min_node_count" {
  description = "Minimum number of nodes in the GKE node pool"
  type        = number
}

variable "max_node_count" {
  description = "Maximum number of nodes in the GKE node pool"
  type        = number
}

variable "oauth2_client" {
  description = "Oauth2 client from https://console.cloud.google.com/apis/credentials"
  type = object({
    client_id = string
    secret = string
  })
}

variable "cookie_secret" {
  description = "Symmetric cookie encryption key for the oauth2-proxy. If null, it will be generated."
  type        = string
}
