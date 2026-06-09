resource "google_compute_global_address" "alb" {
  count      = var.use_alb ? 1 : 0
  name       = "cloud-robotics-alb"
  project    = var.id
  depends_on = [google_project_service.project-services["compute.googleapis.com"]]
}

# Add named port to GKE instance groups
resource "google_compute_instance_group_named_port" "https" {
  count   = var.use_alb ? length(google_container_node_pool.cloud_robotics_base_pool.instance_group_urls) : 0
  group   = replace(google_container_node_pool.cloud_robotics_base_pool.instance_group_urls[count.index], "instanceGroupManagers", "instanceGroups")
  zone    = split("/", google_container_node_pool.cloud_robotics_base_pool.instance_group_urls[count.index])[8]
  name    = "https"
  port    = 32571
  project = var.id
}

resource "google_compute_backend_service" "alb_backend" {
  count                 = var.use_alb ? 1 : 0
  name                  = "nlb-backend"
  project               = var.id
  protocol              = "HTTP2"
  port_name             = "https"
  timeout_sec           = 30
  load_balancing_scheme = "EXTERNAL_MANAGED"

  dynamic "backend" {
    for_each = google_container_node_pool.cloud_robotics_base_pool.instance_group_urls
    content {
      group = replace(backend.value, "instanceGroupManagers", "instanceGroups")
    }
  }

  health_checks = [google_compute_health_check.alb_backend_hc[0].id]

  security_policy = google_compute_security_policy.alb_security_policy[0].id
}

resource "google_compute_health_check" "alb_backend_hc" {
  count   = var.use_alb ? 1 : 0
  name    = "alb-backend-hc"
  project = var.id

  tcp_health_check {
    port = 32571
  }
}

resource "google_compute_managed_ssl_certificate" "alb_cert" {
  count   = var.use_alb ? 1 : 0
  name    = "cloud-robotics-alb-cert"
  project = var.id

  managed {
    domains = [var.domain != "" ? var.domain : "www.endpoints.${var.id}.cloud.goog"]
  }
}

data "kubernetes_secret" "nginx_tls" {
  count = var.use_alb ? 1 : 0
  metadata {
    name      = "tls"
    namespace = "default"
  }
}

resource "google_compute_ssl_certificate" "alb_cert_preprovisioned" {
  count       = var.use_alb ? 1 : 0
  name        = "cloud-robotics-alb-cert-pre"
  project     = var.id
  private_key = data.kubernetes_secret.nginx_tls[0].data["tls.key"]
  certificate = data.kubernetes_secret.nginx_tls[0].data["tls.crt"]
}

resource "google_compute_url_map" "alb_url_map" {
  count           = var.use_alb ? 1 : 0
  name            = "cloud-robotics-alb"
  project         = var.id
  default_service = google_compute_backend_service.alb_backend[0].id
}

resource "google_compute_target_https_proxy" "alb_https_proxy" {
  count            = var.use_alb ? 1 : 0
  name             = "cloud-robotics-alb-proxy"
  project          = var.id
  url_map          = google_compute_url_map.alb_url_map[0].id
  ssl_certificates = [
    google_compute_ssl_certificate.alb_cert_preprovisioned[0].id,
    google_compute_managed_ssl_certificate.alb_cert[0].id
  ]
}


resource "google_compute_global_forwarding_rule" "alb_https_frontend" {
  count      = var.use_alb ? 1 : 0
  name       = "cloud-robotics-alb-https"
  project    = var.id
  target     = google_compute_target_https_proxy.alb_https_proxy[0].id
  port_range = "443"
  ip_address = google_compute_global_address.alb[0].address
}

resource "google_compute_url_map" "https_redirect" {
  count   = var.use_alb ? 1 : 0
  name    = "cloud-robotics-redirect"
  project = var.id

  default_url_redirect {
    https_redirect         = true
    redirect_response_code = "MOVED_PERMANENTLY_DEFAULT"
    strip_query            = false
  }
}

resource "google_compute_target_http_proxy" "alb_http_proxy" {
  count   = var.use_alb ? 1 : 0
  name    = "cloud-robotics-http-proxy"
  project = var.id
  url_map = google_compute_url_map.https_redirect[0].id
}

resource "google_compute_global_forwarding_rule" "alb_http_frontend" {
  count      = var.use_alb ? 1 : 0
  name       = "cloud-robotics-alb-http"
  project    = var.id
  target     = google_compute_target_http_proxy.alb_http_proxy[0].id
  port_range = "80"
  ip_address = google_compute_global_address.alb[0].address
}

resource "google_compute_security_policy" "alb_security_policy" {
  count   = var.use_alb ? 1 : 0
  name    = "cloud-robotics-alb-security-policy"
  project = var.id

  # Default rule (lowest priority)
  rule {
    action   = "allow"
    priority = "2147483647"
    match {
      versioned_expr = "SRC_IPS_V1"
      config {
        src_ip_ranges = ["*"]
      }
    }
    description = "default rule"
  }

  # SQL Injection
  rule {
    action   = "deny(403)"
    priority = "1000"
    match {
      expr {
        expression = "evaluatePreconfiguredExpr('sqli-v33-stable')"
      }
    }
    description = "SQL injection detection"
  }

  # Cross-Site Scripting (XSS)
  rule {
    action   = "deny(403)"
    priority = "1001"
    match {
      expr {
        expression = "evaluatePreconfiguredExpr('xss-v33-stable')"
      }
    }
    description = "XSS detection"
  }

  # Local File Inclusion (LFI)
  rule {
    action   = "deny(403)"
    priority = "1002"
    match {
      expr {
        expression = "evaluatePreconfiguredExpr('lfi-v33-stable')"
      }
    }
    description = "LFI detection"
  }

  # Remote File Inclusion (RFI)
  rule {
    action   = "deny(403)"
    priority = "1003"
    match {
      expr {
        expression = "evaluatePreconfiguredExpr('rfi-v33-stable')"
      }
    }
    description = "RFI detection"
  }

  # Remote Code Execution (RCE)
  rule {
    action   = "deny(403)"
    priority = "1004"
    match {
      expr {
        expression = "evaluatePreconfiguredExpr('rce-v33-stable')"
      }
    }
    description = "RCE detection"
  }

  # Protocol Attacks (helps with HTTP/2 anomalies)
  rule {
    action   = "deny(403)"
    priority = "1005"
    match {
      expr {
        expression = "evaluatePreconfiguredExpr('protocolattack-v33-stable')"
      }
    }
    description = "Protocol attack detection"
  }

  # Scanner Detection
  rule {
    action   = "deny(403)"
    priority = "1006"
    match {
      expr {
        expression = "evaluatePreconfiguredExpr('scannerdetection-v33-stable')"
      }
    }
    description = "Scanner detection"
  }

  # Rate limiting rule to fend against DDoS / Floods (including HTTP/2 multiplexing abuses)
  rule {
    action   = "throttle"
    priority = "2000"
    match {
      expr {
        expression = "true"
      }
    }
    rate_limit_options {
      rate_limit_threshold {
        count        = 500
        interval_sec = 60
      }
      conform_action = "allow"
      exceed_action  = "deny(429)"
      enforce_on_key = "IP"
    }
    description = "Rate limit to prevent floods"
  }
}

data "google_compute_lb_ip_ranges" "ranges" {}

resource "google_compute_firewall" "allow_alb_to_gke" {
  count   = var.use_alb ? 1 : 0
  name    = "allow-alb-to-gke"
  project = var.id
  network = "default"

  allow {
    protocol = "tcp"
    ports    = ["32571"]
  }

  source_ranges           = data.google_compute_lb_ip_ranges.ranges.http_ssl_tcp_internal
  target_service_accounts = [google_service_account.gke_node.email]
}
