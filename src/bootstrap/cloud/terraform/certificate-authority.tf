# Certificate related infrastructure.
#
# Configures the necessary Google private CA infrastructure to create in-cluster certificates.
# Resources in this file are only created if the certificate_provider is set to "google-cas".

resource "google_privateca_ca_pool" "ca_pool" {
  project  = data.google_project.project.project_id
  count    = var.certificate_provider == "google-cas" ? 1 : 0
  name     = "${data.google_project.project.project_id}-ca-pool"
  location = var.region
  # Enterprise is recommended for long-lasting certificates
  tier     = "ENTERPRISE"

  publishing_options {
    publish_ca_cert = true
    publish_crl     = true
  }
}

resource "google_privateca_certificate_authority" "ca" {
  project = data.google_project.project.project_id
  count   = var.certificate_provider == "google-cas" ? 1 : 0
  certificate_authority_id = "${data.google_project.project.project_id}-authority"
  location                 = var.region
  pool                     = google_privateca_ca_pool.ca_pool[0].name
  config {
    subject_config {
      subject {
        organization        = var.certificate_subject_organization
        common_name         = var.certificate_subject_common_name
        organizational_unit = var.certificate_subject_organizational_unit
      }
    }
    x509_config {
      ca_options {
        is_ca = true
        max_issuer_path_length = 10
      }
      key_usage {
        base_key_usage {
          cert_sign = true
          crl_sign  = true
        }
        extended_key_usage {
          server_auth = true
        }
      }
    }
  }
  type = "SELF_SIGNED"
  key_spec {
    algorithm = "EC_P384_SHA384"
  }
}
