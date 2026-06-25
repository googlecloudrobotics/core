# HTTP Relay Benchmarks

This directory contains throughput benchmarks for the HTTP relay (server and client).

## Running Benchmarks

The benchmarks are designed to run using **Bazel**, which handles the complex package structure and dependencies of this workspace.

### Using Bazel

To run all benchmarks with full output:

```bash
bazel test //src/go/tests/relay:throughput_benchmark_test --test_arg=-test.bench=. --test_output=all
```

To run a subset of benchmarks (e.g., only 1MB transfers) in short mode:

```bash
bazel test //src/go/tests/relay:throughput_benchmark_test --test_arg=-test.bench=Size1MB --test_arg=-test.short --test_output=all
```

> **Note:** Standard `go test` is not supported in this directory because it contains multiple packages (`main` and `relay`), a structure that is managed specifically by Bazel.

## Benchmark Results Summary

The benchmarks measure the throughput of transferring data from a backend server through the relay-client and relay-server to a user-client. All components run on `localhost` during these tests.

### Performance Observations

* **Small Transfers (1KB):** Throughput is approximately **2 MB/s**. At this size, the performance is dominated by the HTTP request-response overhead of the relay protocol.
* **Large Transfers (100MB+):** Throughput reaches up to **400 MB/s**.
* **Protocol Overhead:** The relay introduces significant latency and throughput limitations for small payloads due to the multiple round-trips required to poll for requests and post response chunks.

### Configuration Impact

The benchmarks vary two key parameters in the `RelayClient`:

1. **`MaxChunkSize`:** The maximum amount of data accumulated before the relay-client posts a response chunk to the relay-server.
    * **Impact:** This is the most critical setting for throughput. Increasing `MaxChunkSize` from 50KB to 1MB can double the throughput for large files by reducing the number of POST requests from the client to the server.
2. **`BlockSize`:** The size of the internal I/O buffer used when reading from the backend.
    * **Impact:** Larger block sizes provide marginal improvements but are secondary to the chunk size.

### Sample Data (Local Loopback)

| Data Size | Block Size | Max Chunk Size | Throughput |
| :--- | :--- | :--- | :--- |
| 1 KB | 10 KB | 50 KB | ~1.9 MB/s |
| 1 MB | 10 KB | 1 MB | ~180 MB/s |
| 100 MB | 64 KB | 1 MB | ~400 MB/s |

## Design Considerations

Based on this data, future optimizations could include:

* Implementing a more efficient streaming mechanism (e.g., persistent long-lived connections or WebSockets for data transfer) to reduce request overhead.
* Dynamic adjustment of chunk sizes based on available bandwidth and latency.
* Reducing serialization overhead for large payloads.
