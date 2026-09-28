# HTTP Relay Client

The `http-relay-client` runs on the robot cluster and works together with the [`http-relay-server`](../http-relay-server/README.md) in the cloud cluster to expose local HTTP and gRPC backends (such as the robot's Kubernetes API server) to authorized remote clients without requiring a public endpoint or inbound firewall ports on the robot.

## How it works

1. **Long-Polling**: Continuously makes outbound HTTP requests (`/server/request?server=<server_name>`) to the remote `http-relay-server` to pull serialized incoming requests.
2. **Backend Forwarding**: Deserializes each request and forwards it to the configured local backend server (`--backend_scheme` / `--backend_address` / `--backend_path`), optionally injecting a local authentication bearer token (`--authentication_token_file`).
3. **Response & Stream Relaying**: Captures the backend's HTTP response (including streaming responses, HTTP/2, gRPC, and `101 Switching Protocols` upgrades such as `kubectl exec` or WebSockets) and posts the serialized chunks back to the `http-relay-server` (`/server/response`).

For the complete architecture, sequence diagrams, and scalability considerations, see the [`http-relay-server` README](../http-relay-server/README.md).

## Flags

- `--backend_scheme`: Connection scheme (`http`, `https`) from relay client to local backend server (default: `"https"`).
- `--backend_address`: Hostname and port of the local backend server (default: `"localhost:8080"`).
- `--backend_path`: Optional path prefix added to backend requests (default: `""`).
- `--preserve_host`: Preserve the `Host` header of the original request for compatibility with cross-origin checks (default: `false`).
- `--relay_scheme`: Connection scheme (`http`, `https`) from relay client to the remote relay server (default: `"https"`).
- `--relay_address`: Hostname and port of the remote relay server (default: `"localhost:8081"`).
- `--relay_prefix`: Path prefix for the relay server endpoints (default: `""`).
- `--server_name`: Unique identifier under which this relay client registers and fetches requests from the relay server (default: `"foo"`).
- `--authentication_token_file`: Optional file containing a bearer token injected into local backend requests (default: `""`).
- `--root_ca_file`: Optional root CA certificate file for TLS verification (default: `""`).
- `--max_chunk_size`: Maximum size of data in bytes to accumulate before sending to the peer (default: `51200`).
- `--block_size`: Size of I/O buffer in bytes (default: `10240`).
- `--num_pending_requests`: Number of concurrent pending poll requests to maintain with the relay server (default: `1`).
- `--max_idle_conns_per_host`: Maximum number of idle (keep-alive) connections to keep per host (default: `50`).
- `--disable_http2`: Disable HTTP/2 protocol usage (e.g. for channels that require HTTP/1.1 protocol upgrades such as SPDY) (default: `false`).
- `--force_http2`: Force HTTP/2 transport usage (e.g. when relaying cleartext gRPC) (default: `false`).
- `--disable_auth_for_remote`: Disable authentication when talking to the relay server for local testing (default: `false`).
- `--trace-stackdriver-project-id`: If non-empty, uploads OpenTelemetry traces to this Google Cloud Project (default: `""`).
- `--log_level`: Minimum log message level required to be logged (default: `0` / `INFO`).
- `--pprof_port`: If non-zero, serves Go `pprof` profiling endpoints on this port (default: `0`).
