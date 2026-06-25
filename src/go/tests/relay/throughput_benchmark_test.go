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

package main

import (
	"fmt"
	"io"
	"net/http"
	"testing"

	"github.com/googlecloudrobotics/core/src/go/cmd/http-relay-client/client"
	rt "github.com/googlecloudrobotics/core/src/go/tests/relay"
)

type zeroReader struct{}

func (r *zeroReader) Read(p []byte) (int, error) {
	for i := range p {
		p[i] = 0
	}
	return len(p), nil
}

func BenchmarkThroughput(b *testing.B) {
	sizes := []struct {
		name string
		size int64
	}{
		{"1KB", 1024},
		{"1MB", 1024 * 1024},
		{"10MB", 10 * 1024 * 1024},
		{"100MB", 100 * 1024 * 1024},
	}

	if !testing.Short() {
		sizes = append(sizes, struct {
			name string
			size int64
		}{"1GB", 1024 * 1024 * 1024})
	}

	blockSizes := []int{10 * 1024, 64 * 1024, 256 * 1024}
	maxChunkSizes := []int{50 * 1024, 256 * 1024, 1024 * 1024}

	for _, s := range sizes {
		for _, bs := range blockSizes {
			for _, mcs := range maxChunkSizes {
				b.Run(fmt.Sprintf("Size%s/Block%d/Chunk%d", s.name, bs, mcs), func(b *testing.B) {
					config := client.DefaultClientConfig()
					config.BlockSize = bs
					config.MaxChunkSize = mcs
					config.ServerName = "bench-server"
					config.DisableAuthForRemote = true
					config.RelayScheme = "http"
					config.BackendScheme = "http"

					env, backendLn, cleanup := rt.SetupRelay(b, config)
					defer cleanup()

					mux := http.NewServeMux()
					mux.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
						w.Header().Set("Content-Type", "application/octet-stream")
						io.CopyN(w, &zeroReader{}, s.size)
					})
					backendServer := &http.Server{Handler: mux}
					go backendServer.Serve(backendLn)
					defer backendServer.Close()

					rt.WaitForClient(b, env.RelayPort, config.ServerName)

					relayURL := fmt.Sprintf("http://127.0.0.1:%d/client/%s/", env.RelayPort, config.ServerName)

					b.ResetTimer()
					for i := 0; i < b.N; i++ {
						resp, err := http.Get(relayURL)
						if err != nil {
							b.Fatal(err)
						}
						n, err := io.Copy(io.Discard, resp.Body)
						resp.Body.Close()
						if err != nil {
							b.Fatal(err)
						}
						if n != s.size {
							b.Fatalf("expected %d bytes, got %d", s.size, n)
						}
					}
					b.SetBytes(s.size)
				})
			}
		}
	}
}
