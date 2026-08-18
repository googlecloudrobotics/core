# VictoriaMetrics - Cloud Metrics (`victoriametrics-cloudmetrics`)

## Overview

`victoriametrics-cloudmetrics` is the observability app chart dedicated exclusively to **Cloud Service and Infrastructure Telemetry** (metrics originating within the cloud platform and Kubernetes control plane).

Unlike `victoriametrics-robotmetrics`, this chart is **cloud-only** and has no edge or robot component.

---

## Architecture Diagram

```mermaid
flowchart TD
    subgraph CloudCluster[Cloud Platform - Deployment Target: cloud]
        KSM[kube-state-metrics] -->|Scraped by| CA[vmagent - Cloud Agent]
        CS[Cloud Microservices] -->|Scraped by| CA
        K8S[Kubernetes Control Plane] -->|Scraped by| CA

        CA -->|Writes| VMI[vminsert - Ingestion Layer]
        VMI --> VMS[vmstorage - Storage Layer]
        VMQ[vmselect - Query Engine] --> VMS
        VMA[vmalert - Alert Manager] --> VMQ
    end
```

---

## Key Terminology

* **Telemetry Domain**: `cloudmetrics` (What is measured: metrics originating from cloud services and Kubernetes infrastructure).
* **Deployment Target**: `cloud` (Cloud-only deployment; no robot edge agent).
