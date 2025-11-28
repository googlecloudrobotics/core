# HTTP Relay Server

The HTTP relay server multiplexes HTTP requests between user-clients and backends (robots) via a relay-client.
This allows multiple backends to be accessible through a single relay server instance, and supports multiple concurrent user-clients.

## Flags

*   `--port`: Port number to listen on (default: 80).
*   `--block_size`: Size of i/o buffer in bytes (default: 10240).
*   `--inactive_request_timeout`: Timeout for inactive requests (default: 60s). In particular, this sets a limit on how long the backend can wait before writing headers and the response status.

## Configuration

### Nginx Timeout

If you are running the relay server behind Nginx, ensure that the proxy read timeout on Nginx is set such that Nginx doesn't time out before the http-relay-server does.

Specifically, the `nginx.ingress.kubernetes.io/proxy-read-timeout` annotation (or `proxy_read_timeout` directive in nginx config) should be set to a value larger than `--inactive_request_timeout`.

For example, if `--inactive_request_timeout` is set to `60s`, you might set `nginx.ingress.kubernetes.io/proxy-read-timeout` to `75s`.
