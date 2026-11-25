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

// Package httprelay adapts WebSocket connections into net.Conn byte streams
// and provides shared yamux configuration for the HTTP relay tunnel.
package httprelay

import (
	"errors"
	"fmt"
	"io"
	"log/slog"
	"net"
	"sync"
	"time"

	"github.com/gorilla/websocket"
	"github.com/hashicorp/yamux"
)

const (
	// yamuxKeepAliveInterval sends yamux PING frames below common 30s L7 proxy
	// and edge tunnel idle timeouts.
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

// WSConn adapts a gorilla/websocket connection to net.Conn using binary frames.
type WSConn struct {
	ws *websocket.Conn

	readMu sync.Mutex
	reader io.Reader

	writeMu sync.Mutex
}

// NewWSConn wraps ws as a net.Conn backed by binary WebSocket frames.
func NewWSConn(ws *websocket.Conn) *WSConn {
	return &WSConn{ws: ws}
}

// Read reads bytes from binary WebSocket frames, returning io.EOF on clean
// WebSocket closure.
func (c *WSConn) Read(p []byte) (int, error) {
	c.readMu.Lock()
	defer c.readMu.Unlock()

	for {
		if c.reader == nil {
			msgType, r, err := c.ws.NextReader()
			if err != nil {
				return 0, normalizeCloseErr(err)
			}
			if msgType != websocket.BinaryMessage {
				return 0, fmt.Errorf("unexpected websocket message type %d", msgType)
			}
			c.reader = r
		}
		n, err := c.reader.Read(p)
		if errors.Is(err, io.EOF) {
			c.reader = nil
			if n > 0 {
				return n, nil
			}
			continue
		}
		return n, err
	}
}

// Write sends p as a binary WebSocket message.
func (c *WSConn) Write(p []byte) (int, error) {
	c.writeMu.Lock()
	defer c.writeMu.Unlock()

	if err := c.ws.WriteMessage(websocket.BinaryMessage, p); err != nil {
		return 0, err
	}
	return len(p), nil
}

// Close closes the underlying WebSocket connection.
func (c *WSConn) Close() error {
	return c.ws.Close()
}

// LocalAddr returns the local network address of the WebSocket connection.
func (c *WSConn) LocalAddr() net.Addr {
	return c.ws.LocalAddr()
}

// RemoteAddr returns the remote network address of the WebSocket connection.
func (c *WSConn) RemoteAddr() net.Addr {
	return c.ws.RemoteAddr()
}

// SetDeadline sets both read and write deadlines on the WebSocket connection.
func (c *WSConn) SetDeadline(t time.Time) error {
	if err := c.ws.SetReadDeadline(t); err != nil {
		return err
	}
	return c.ws.SetWriteDeadline(t)
}

// SetReadDeadline sets the read deadline on the WebSocket connection.
func (c *WSConn) SetReadDeadline(t time.Time) error {
	return c.ws.SetReadDeadline(t)
}

// SetWriteDeadline sets the write deadline on the WebSocket connection.
func (c *WSConn) SetWriteDeadline(t time.Time) error {
	return c.ws.SetWriteDeadline(t)
}

func normalizeCloseErr(err error) error {
	if websocket.IsCloseError(err, websocket.CloseNormalClosure, websocket.CloseGoingAway) {
		return io.EOF
	}
	return err
}
