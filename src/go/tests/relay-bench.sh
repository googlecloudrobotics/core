#!/bin/bash
# Runs a local http server, a local relay server + client and compares direct 
# access to relayed access.

trap suite_cleanup INT

backend_server_port=8082
backend_server="http://localhost:${backend_server_port}"
relay_server_port=8081
relay_server="http://localhost:${relay_server_port}"
server_name="test"

# per suite
function suite_init() {
  sudo cpufreq-set -g performance
  bazel build //src/go/cmd/http-relay-client:http-relay-client-bin //src/go/cmd/http-relay-server:http-relay-server-bin
  which >/dev/null go-httpbin || die "Please run: go install github.com/mccutchen/go-httpbin/v2/cmd/go-httpbin"
}

function suite_cleanup() {
  sudo cpufreq-set -g ondemand
  test_cleanup
}

# per test
function test_init() {
  go-httpbin >/tmp/backend_server.log 2>&1 \
    -host 127.0.0.1 -port 8082 &
  backend_server_pid=$!
  bazel-bin/src/go/cmd/http-relay-server/http-relay-server-bin_/http-relay-server-bin >/tmp/relay_server.log 2>&1 \
    --port=${relay_server_port} &
  relay_server_pid=$!
  # ensure server is up
  sleep 1s
  bazel-bin/src/go/cmd/http-relay-client/http-relay-client-bin_/http-relay-client-bin  >/tmp/relay_client.log 2>&1 \
    --backend_address=localhost:${backend_server_port} --backend_scheme=http --relay_scheme=http --server_name="${server_name}" "$@" &
  relay_client_pid=$!
}

function test_cleanup() {
  test -n "$relay_client_pid" && kill $relay_client_pid && unset relay_client_pid
  test -n "$relay_server_pid" && kill $relay_server_pid && unset relay_server_pid
  test -n "$backend_server_pid" && kill $backend_server_pid && unset backend_server_pid
}

# helper
function die {
  echo "$1" >&2
  exit 1
}

function get_avg() {
  awk -F',' '{sum+=$7} END {print sum/NR}' $1
}

function status() {
  printf "%s: direct=%9.7fs, relay=%9.7fs, slowdown=%9.7fs\n" $1 $2 $3 $(echo $3/$2 | bc -l)
}

function curltime() {
    curl -w @- -o /dev/null -s "$@" <<'EOF'
%{time_namelookup},%{time_connect},%{time_appconnect},%{time_pretransfer},%{time_redirect},%{time_starttransfer},%{time_total}\n
EOF
}

# benchmarks
function run_test() {
  test_name="$1"
  shift
  echo "==== $test_name : $@"
  local num_runs
  num_runs=$1
  shift
  local req_path
  req_path="$1"
  shift
  test_init "$@"
  
  (for i in $(seq $num_runs); do curltime ${backend_server}/${req_path}; done) >/tmp/direct.seq.csv
  direct=$(get_avg /tmp/direct.seq.csv)
  (for i in $(seq $num_runs); do curltime ${relay_server}/${req_path} -H"X-Server-Name: ${server_name}"; done) >/tmp/relay.seq.csv
  relay=$(get_avg /tmp/relay.seq.csv)
  status "seq" ${direct} ${relay}


  (for i in $(seq $num_runs); do curltime ${backend_server}/${req_path} & done) >/tmp/direct.par.csv; 
  sleep 1s
  direct=$(get_avg /tmp/direct.par.csv)
  (for i in $(seq $num_runs); do curltime ${relay_server}/${req_path} -H"X-Server-Name: ${server_name}" & done) >/tmp/relay.par.csv;
  sleep 1s
  relay=$(get_avg /tmp/relay.par.csv)
  status "par" ${direct} ${relay}
  
  test_cleanup
}

function run() {
  suite_init
  local num_runs
  num_runs=100
  run_test "   default_params" $num_runs "bytes/100000"
  run_test "    more_requests" $num_runs "bytes/100000" --max_idle_conns_per_host=100 --num_pending_requests=10
  run_test "    more_requests" $num_runs "bytes/100000" --max_idle_conns_per_host=100 --num_pending_requests=50
  suite_cleanup
}

if [[ -z "$1" ]]; then
  run
else
  # call arguments verbatim:
  "$@"
fi

