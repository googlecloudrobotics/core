// Copyright 2026 The Cloud Robotics Authors
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

// Package httprelay provides shared yamux configuration for the HTTP relay
// WebSocket tunnel.
package httprelay

import (
	"fmt"
	"log/slog"
	"time"

	"github.com/hashicorp/yamux"
)

const (
	// yamuxKeepAliveInterval sends yamux PING frames below common 30s L7 proxy
	// and tunnel idle timeouts.
	yamuxKeepAliveInterval = 20 * time.Second
	// yamuxMaxStreamWindowSize raises the per-stream flow control window from
	// yamux's 256 KiB default to sustain high throughput over WAN RTTs.
	yamuxMaxStreamWindowSize = 4 << 20
)

// YamuxConfig returns the shared yamux session configuration used by both
// http-relay-server and http-relay-client.
func YamuxConfig() *yamux.Config {
	cfg := yamux.DefaultConfig()
	cfg.EnableKeepAlive = true
	cfg.KeepAliveInterval = yamuxKeepAliveInterval
	cfg.MaxStreamWindowSize = yamuxMaxStreamWindowSize
	cfg.LogOutput = nil
	cfg.Logger = yamuxLogger{}
	return cfg
}

type yamuxLogger struct{}

func (yamuxLogger) Print(v ...any) {
	slog.Debug("yamux", slog.String("msg", fmt.Sprint(v...)))
}

func (yamuxLogger) Printf(format string, v ...any) {
	slog.Debug("yamux", slog.String("msg", fmt.Sprintf(format, v...)))
}

func (yamuxLogger) Println(v ...any) {
	slog.Debug("yamux", slog.String("msg", fmt.Sprintln(v...)))
}
