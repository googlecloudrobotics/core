# Status Echo Server

A simple Go HTTP server that echoes back a configured or requested HTTP status code.

## Behavior

The server listens on a configured port and handles all HTTP requests by returning a text response with the requested HTTP status code.

The status code returned is determined in the following order of precedence:

1.  **URL Path Suffix**: If the path ends with `/status/<3-digit-code>` (e.g., `/status/201`), that code is used.
2.  **Query Parameter**: If no valid path suffix is found, it looks for the `status` query parameter (e.g., `/?status=202`).
3.  **Default Flag**: If neither is provided, it defaults to the value set by the `-http_status` CLI flag (which defaults to `200`).

### Validation

If a status code is requested via path or query but is invalid (e.g., not a number, or not a recognized HTTP status code like `999`), the server logs a warning and returns `400 Bad Request`.

---

## Running Locally

To build and run the server locally using Bazel:

```bash
bazel run //src/go/cmd/status-echo-server:status-echo-server-app -- -port 8080
```

---

## Running Tests

To run the unit tests locally using Bazel:

```bash
bazel test :go_default_test
```

### Useful Test Flags

*   **Disable caching** (force tests to run even if code didn't change):
    ```bash
    bazel test :go_default_test --nocache_test_results
    ```
*   **Show verbose output** (print test logs to terminal even if they pass):
    ```bash
    bazel test :go_default_test --test_output=all
    ```
*   **Run a specific test**:
    ```bash
    bazel test :go_default_test --test_arg=-test.run=TestEchoHandler/OK_status_on_root_path
    ```

---

## Running Benchmarks

To run the Go micro-benchmarks locally using Bazel:

```bash
bazel test :go_default_test --test_arg=-test.bench=. --test_arg=-test.run=^$ --test_output=all --test_arg=-test.benchmem 
```

### Useful Benchmark Flags

*   **Disable caching** (force benchmarks to run):
    ```bash
    bazel test :go_default_test --nocache_test_results --test_output=all --test_arg=-test.bench=. --test_arg=-test.run=^$ --test_arg=-test.benchmem 
    ```
*   **Run for a specific duration** (default is 1s):
    ```bash
    bazel test :go_default_test --test_arg=-test.benchtime=5s --test_arg=-test.bench=. --test_arg=-test.run=^$ --test_output=all --test_arg=-test.benchmem 
    ```