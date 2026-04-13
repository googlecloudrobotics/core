# HTTP Relay Server

The http-relay-server multiplexes HTTP requests between user-clients and backends (robots) via a relay-client.
This allows multiple backends to be accessible through a single relay server instance, and supports multiple concurrent user-clients.

## Tested capabilities

The http-relay-server was originally designed as a way to use kubectl against remote clusters.
It traverses firewalls by only making outbound requests to the public internet from both the user client (eg kubectl, browser) and the remote cluster.
It has been tested with the following traffic:

- HTTP 1.1 & 2 from web browsers (including bidirectional streaming with websockets)
- HTTP 1.1 from kubectl, including streaming response bodies for `kubectl logs`
- SPDY from kubectl (via HTTP 101 Switching Protocols) for `kubectl exec`
- unidirectional gRPC (HTTP2 cleartext and HTTP trailers)

The following is known not to work:

- streaming gRPC fails with `rpc error: code = Internal desc = stream terminated by RST_STREAM with error code: PROTOCOL_ERROR`, root cause unknown

The following has not been tested:

- HTTP 1.1 streaming request body (`Transfer-Encoding: chunked` in the request header)

## Flags

*   `--port`: Port number to listen on (default: 80).
*   `--block_size`: Size of i/o buffer in bytes (default: 10240).
*   `--inactive_request_timeout`: Timeout for inactive requests (default: 60s). In particular, this sets a limit on how long the backend can wait before writing headers and the response status.

## Configuration

### Nginx Timeout

If you are running the relay server behind Nginx, ensure that the proxy read timeout on Nginx is set such that Nginx doesn't time out before the http-relay-server does.

Specifically, the `nginx.ingress.kubernetes.io/proxy-read-timeout` annotation (or `proxy_read_timeout` directive in nginx config) should be set to a value larger than `--inactive_request_timeout`.

For example, if `--inactive_request_timeout` is set to `60s`, you might set `nginx.ingress.kubernetes.io/proxy-read-timeout` to `75s`.
