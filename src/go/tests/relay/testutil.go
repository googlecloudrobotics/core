// Copyright 2023 The Cloud Robotics Authors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package relay

import (
	"bytes"
	"context"
	"fmt"
	"io"
	"net"
	"net/http"
	"testing"
	"time"

	"github.com/googlecloudrobotics/core/src/go/cmd/http-relay-client/client"
	"github.com/googlecloudrobotics/core/src/go/cmd/http-relay-server/server"
)

// listenAndPort opens a TCP listener on a random free port on 127.0.0.1.
func listenAndPort(t testing.TB) (net.Listener, int) {
	t.Helper()
	ln, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatalf("failed to listen: %v", err)
	}
	return ln, ln.Addr().(*net.TCPAddr).Port
}

// waitForRelay waits for the relay server at the given port to become healthy.
func waitForRelay(t testing.TB, relayPort int) {
	t.Helper()
	deadline := time.Now().Add(10 * time.Second)
	for {
		if time.Now().After(deadline) {
			t.Fatal("timeout waiting for relay server healthz")
		}
		res, err := http.Get(fmt.Sprintf("http://127.0.0.1:%d/healthz", relayPort))
		if err == nil {
			res.Body.Close()
			if res.StatusCode == http.StatusOK {
				break
			}
		}
		time.Sleep(100 * time.Millisecond)
	}
}

// WaitForClient waits for the relay client to register with the relay server.
func WaitForClient(t testing.TB, relayPort int, serverName string) {
	t.Helper()
	relayAddress := fmt.Sprintf("http://127.0.0.1:%d/client/%s/", relayPort, serverName)
	deadline := time.Now().Add(10 * time.Second)
	for {
		if time.Now().After(deadline) {
			t.Fatal("timeout waiting for relay client registration")
		}
		res, err := http.Get(relayAddress)
		if err == nil {
			body, _ := io.ReadAll(res.Body)
			res.Body.Close()
			if res.StatusCode == http.StatusServiceUnavailable && bytes.Contains(body, []byte("unknown client")) {
				time.Sleep(100 * time.Millisecond)
				continue
			}
			break
		}
		time.Sleep(100 * time.Millisecond)
	}
}

// startRelay starts a relay server on the given listener and a relay client with the given config.
func startRelay(t testing.TB, ctx context.Context, relayLn net.Listener, config client.ClientConfig) {
	t.Helper()
	relayServer := server.NewServer(server.Config{
		BlockSize: 10 * 1024,
	})
	go relayServer.StartOnListener(ctx, relayLn)

	relayClient := client.NewClient(config)
	go relayClient.Start(ctx)
}

// RelayEnv holds the configuration for a relay test environment.
type RelayEnv struct {
	RelayPort   int
	BackendPort int
}

// SetupRelay creates a new relay environment with a relay server and client.
// It returns the environment, a listener for the backend server, and a cleanup function.
func SetupRelay(t testing.TB, config client.ClientConfig) (*RelayEnv, net.Listener, func()) {
	t.Helper()

	backendLn, backendPort := listenAndPort(t)
	relayLn, relayPort := listenAndPort(t)

	config.RelayAddress = fmt.Sprintf("127.0.0.1:%d", relayPort)
	config.BackendAddress = fmt.Sprintf("127.0.0.1:%d", backendPort)

	ctx, cancel := context.WithCancel(context.Background())
	startRelay(t, ctx, relayLn, config)
	waitForRelay(t, relayPort)

	cleanup := func() {
		cancel()
		backendLn.Close()
		// Do not close relayLn here; server.StartOnListener handles it via Shutdown.
	}

	return &RelayEnv{
		RelayPort:   relayPort,
		BackendPort: backendPort,
	}, backendLn, cleanup
}
