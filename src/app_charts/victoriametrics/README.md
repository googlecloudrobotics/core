# VictoriaMetrics Telemetry Architecture

## Overview

VictoriaMetrics in this repository is split into two distinct observability domains to isolate high-cardinality, potentially unstable network edge telemetry from internal cloud infrastructure telemetry.

Both components live under this directory:

1. **`victoriametrics-robotmetrics`**: 
   * **Domain**: Robot telemetry (data originating from physical edge robots).
   * **Deployment Targets**: Deploys `vmagent` on edge robots, and a complete `vmcluster` in the cloud to receive, store, and alert on robot data.
2. **`victoriametrics-cloudmetrics`**:
   * **Domain**: Cloud telemetry (data originating from cloud microservices and Kubernetes infrastructure).
   * **Deployment Target**: Cloud-only. Scrapes `kube-state-metrics`, node exporters, and cloud services into a dedicated internal `vmcluster`.

---

## Unified Architecture Diagram

```mermaid
flowchart TD
    subgraph Edge[Edge Environment]
        RA[vmagent - Robot Agent] -->|Scrapes| RT[Robot Local Services]
    end
    
    subgraph LegacyEdge[Legacy / Dangling Edge]
        LP[Prometheus - Legacy Agent]
    end

    subgraph CloudEnv[Cloud Platform Environment]
        subgraph RobotMetrics[victoriametrics-robotmetrics]
            ING[Ingress / Auth Proxy]
            VMA_PROXY[vmagent - Cloud Ingestion Proxy]
            VMI_R[vminsert - Robot]
            VMS_R[vmstorage - Robot]
            VMQ_R[vmselect - Robot]
            VMA_R[vmalert - Robot]
            
            ING --> VMA_PROXY
            VMA_PROXY --> VMI_R
            VMA_PROXY -.->|Optional Upstream Relay| Mimir[Cloud-Ops Mimir]
            VMI_R --> VMS_R
            VMQ_R --> VMS_R
            VMA_R --> VMQ_R
        end
        
        subgraph CloudMetrics[victoriametrics-cloudmetrics]
            KSM[kube-state-metrics]
            CA[vmagent - Cloud Agent]
            VMI_C[vminsert - Cloud]
            VMS_C[vmstorage - Cloud]
            VMQ_C[vmselect - Cloud]
            VMA_C[vmalert - Cloud]
            
            KSM -->|Scraped by| CA
            CA --> VMI_C
            VMI_C --> VMS_C
            VMQ_C --> VMS_C
            VMA_C --> VMQ_C
        end
        
        CP[Cloud Prometheus Relay]
    end

    %% Data Flow
    RA -->|TLS remoteWrite| ING
    LP -->|Scraped by Cloud Prom| CP
    CP -->|remoteWrite Relay| VMI_R
```

## Documentation

* [Robot Metrics README](victoriametrics-robotmetrics/README.md)
* [Cloud Metrics README](victoriametrics-cloudmetrics/README.md)
