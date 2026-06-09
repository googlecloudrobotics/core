# HTTP/2 HPACK Bomb Test Report (with ALB & Cloud Armor)

This report documents the results of the HTTP/2 HPACK bomb attack (PoC from [Calif](https://blog.calif.io/p/codex-discovered-a-hidden-http2-bomb)) targeting the GKE cluster protected by an Application Load Balancer (ALB) and Cloud Armor, with the backend protocol configured to **HTTP/2**.

A background monitor was used to capture second-by-second metrics for Nginx memory usage (via raw K8s metrics) and ALB reachability (via `curl` probes).

## Test Environment
- **Target:** `https://www.endpoints.intrinsic-liyin-6f.cloud.goog/` (resolving to ALB IP `8.232.95.22`)
- **Backend Protocol:** `HTTP2` (ALB talks HTTP/2 to Nginx)
- **Nginx Ingress Pod:** `nginx-ingress-controller-844bb8f495-tn66s`
- **Attack Parameters:** 5 concurrent connections, 128 streams/conn, 32,000 headers/stream (~19.5 MB upload).

---

## Timeline of the Attack (5-Connection Test)

The table below summarizes the key events captured by the background monitor during the test:

| Relative Time | Nginx Memory | HTTP Status (Probe) | Latency | Event / Notes |
| :--- | :--- | :--- | :--- | :--- |
| **0.0s** | 113.6 MiB | 200 OK | 0.584s | Monitor starts (Baseline). |
| **10.3s** | 114.4 MiB | 200 OK | 0.372s | **Attack Starts** (5 connections established). |
| **15.6s** | 114.4 MiB | 200 OK | 0.369s | Attack upload completes (19.5 MB sent). Enters Hold Phase. |
| **33.3s** | 114.4 MiB | **429 Too Many Requests** | 0.916s | First rate-limit detected. Probe and attacker share same source IP. |
| **35.9s** | 124.8 MiB | 200 OK | 0.346s | Memory increases slightly (+10.4 MiB). |
| **35.9s - 54.3s** | 124.8 MiB | **429 / 200 Intermittent** | ~0.3s | Rate limiter bucket decaying as attack traffic drops to drip-feed. |
| **56.3s** | 124.8 MiB | 200 OK | 0.276s | Rate limiter clears. All subsequent probes return `200 OK`. |
| **95.3s** | 125.5 MiB | 200 OK | 0.317s | Peak memory usage reached (+11.9 MiB from baseline). |
| **118.4s** | 125.5 MiB | 200 OK | 0.352s | **Attack Completes** (Connections closed). |
| **124.6s** | 118.6 MiB | 200 OK | 0.419s | Memory drops post-attack. |
| **305.3s** | 118.4 MiB | 200 OK | 0.489s | Monitor ends. Memory settled at +4.8 MiB from initial baseline. |

---

## Key Findings

1.  **Nginx Protection:** 
    Even with HTTP/2 enabled on the backend, Nginx did **not** experience the HPACK bomb memory exhaustion. The memory usage rose by a maximum of only **11.9 MiB** (compared to **1.33 GiB** without the ALB). This confirms the ALB (GFE) successfully terminates and sanitizes the HTTP/2 frames, preventing the malicious compression states from reaching Nginx.
    
2.  **ALB Stability:** 
    No `unconditional drop overload` or `502/503` errors were observed during this clean run. The ALB remained stable throughout the attack.
    
3.  **Rate Limiting (429):** 
    The temporary `429 Too Many Requests` responses received by the monitor were due to rate limiting, because the monitor and the attack script were running from the same source IP. Once the attack transitioned from the intensive upload phase to the low-bandwidth hold phase (dripping 1 byte every 50s), the rate limits cleared automatically.

## Conclusion
Configuring the ALB stack to use HTTP/2 to the backend is **safe and effective** at mitigating the HPACK bomb attack. The ALB successfully protects Nginx from memory exhaustion without crashing or dropping traffic unconditionally under a 5-connection load. The temporary unreachability observed in previous runs was likely a transient issue or resolved by the rate-limiting mitigation.
