# VictoriaMetrics - Robot Metrics (`victoriametrics-robotmetrics`)

## Overview

`victoriametrics-robotmetrics` is the observability app chart dedicated to **Robot Telemetry** (metrics originating from edge robots). 

Because robot telemetry spans across the network boundary between edge devices and the cloud platform, this chart defines two distinct deployment targets:

1. **Robot Target (`values-robot.yaml`)**: Deploys lightweight edge agents (`vmagent`) directly onto edge robots to scrape local targets.
2. **Cloud Target (`values-cloud.yaml`)**: Deploys the central ingestion, storage, and alerting cluster (`vmcluster`, `vmalert`) in the cloud.

Additionally, for backward compatibility during migration, the cloud Prometheus instance relays metrics from legacy or un-upgraded robots directly into the cloud `vminsert` endpoint via `remoteWrite`.

---

## Architecture Diagram

```mermaid
flowchart TD
    subgraph EdgeRobot[Edge Robot - Deployment Target: robot]
        RA[vmagent - Robot Agent] -->|Scrapes| RT[Local Robot Targets]
    end

    subgraph LegacyRobot[Dangling or Legacy Robot]
        LP[Prometheus - Legacy Stack]
    end

    subgraph CloudPlatform[Cloud Platform - Deployment Target: cloud]
        CP[Cloud Prometheus Relay]
        ING[Ingress or Auth Proxy]
        VMA_PROXY[vmagent - Cloud Ingestion Proxy]
        VMI[vminsert - Ingestion Layer]
        VMS[vmstorage - Storage Layer]
        VMQ[vmselect - Query Engine]
        VMA[vmalert - Alert Manager]

        ING -->|Routes| VMA_PROXY
        VMA_PROXY --> VMI
        VMA_PROXY -.->|Optional Relays Data| Mimir[Cloud-Ops Mimir]
        VMI --> VMS
        VMQ --> VMS
        VMA --> VMQ
    end

    RA -->|remoteWrite over TLS| ING
    LP -->|Scraped by Cloud Prom| CP
    CP -->|remoteWrite Relay| VMI
```

---

## Key Terminology

* **Telemetry Domain**: `robotmetrics` (What is measured: telemetry originating from edge robots).
* **Deployment Targets**:
  * `robot`: Deploys `vmagent` on the physical robot.
  * `cloud`: Deploys `vmcluster` (`vminsert`, `vmstorage`, `vmselect`, `vmalert`) in the cloud.
