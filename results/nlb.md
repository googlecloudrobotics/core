# HTTP/2 Bomb Test Results

**Date:** 2026-06-09
**Target:** `www.endpoints.intrinsic-liyin-6f.cloud.goog`
**Target Pod:** `nginx-ingress-controller-7f64d5c796-4vxbl` (Nginx Ingress Controller v1.8.4, Bundled Nginx 1.21.6)

## Test Scenarios and Results

### Scenario 1: Single Connection Demo
*   **Connections:** 1
*   **Streams per Connection:** 128
*   **Headers per Stream:** 32,000
*   **Hold Time:** 60s
*   **Estimated Server Memory:** ~270 MiB
*   **Measured Memory:**
    *   **Baseline:** 106 MiB
    *   **Peak (During Attack):** 373 MiB (Increase of **267 MiB**)
    *   **Post-Attack (After Connection Close):** 108 MiB (Returned to baseline)

### Scenario 2: Multi-Connection Attack (5 Connections)
*   **Connections:** 5
*   **Streams per Connection:** 128
*   **Headers per Stream:** 32,000
*   **Hold Time:** 60s
*   **Estimated Server Memory:** ~1.3 GiB
*   **Measured Memory:**
    *   **Baseline:** 108 MiB
    *   **Peak (During Attack):** 1441 MiB (Increase of **1.33 GiB**)
    *   **Post-Attack (After Connection Close):** 1441 MiB (Memory **remained allocated** permanently)

## Conclusion
The Nginx Ingress Controller in the cluster is **vulnerable** to the HTTP/2 Bomb attack (CVE-2026-49975).
While a single connection attack allowed memory to be reclaimed, a moderate attack with 5 connections resulted in **permanent memory retention (RSS leak)** of ~1.3 GiB, degrading the worker node's available memory even after connections were terminated.
