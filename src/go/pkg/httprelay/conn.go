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

// Package httprelay adapts bidirectional gRPC byte streams into net.Conn
// connections for HTTP reverse proxying.
package httprelay

import (
	"net"
	"os"
	"sync"
	"time"
)

// MaxChunkSize caps each gRPC message payload well below gRPC's 4 MiB default
// receive limit.
const MaxChunkSize = 256 * 1024

// StreamFuncs holds the underlying stream operations for a StreamConn.
type StreamFuncs struct {
	Send       func([]byte) error
	Recv       func() ([]byte, error)
	CloseWrite func() error
	Close      func() error
}

type readResult struct {
	data []byte
	err  error
}

// StreamConn adapts a bidirectional byte stream into a net.Conn.
type StreamConn struct {
	fns StreamFuncs

	readMu       sync.Mutex
	pending      []byte
	readErr      error
	readCh       chan readResult
	readDeadline deadline

	writeMu        sync.Mutex
	writeDeadline  deadline
	closeWriteOnce sync.Once
	closeWriteErr  error

	closeOnce sync.Once
	closeErr  error
	closed    chan struct{}
}

// NewStreamConn creates a StreamConn, seeding any already-received initial
// bytes ahead of subsequent Recv calls.
func NewStreamConn(initial []byte, fns StreamFuncs) *StreamConn {
	c := &StreamConn{
		fns:           fns,
		pending:       initial,
		readCh:        make(chan readResult),
		readDeadline:  newDeadline(),
		writeDeadline: newDeadline(),
		closed:        make(chan struct{}),
	}
	go c.recvLoop()
	return c
}

func (c *StreamConn) recvLoop() {
	for {
		data, err := c.fns.Recv()
		if err == nil && len(data) == 0 {
			// Empty messages serve as proxy keepalives.
			continue
		}
		select {
		case c.readCh <- readResult{data: data, err: err}:
		case <-c.closed:
			return
		}
		if err != nil {
			return
		}
	}
}

// Read reads bytes from the stream, honoring SetReadDeadline without dropping
// unread stream messages.
func (c *StreamConn) Read(p []byte) (int, error) {
	c.readMu.Lock()
	defer c.readMu.Unlock()

	if isClosedChan(c.readDeadline.wait()) {
		return 0, os.ErrDeadlineExceeded
	}
	if len(c.pending) > 0 {
		n := copy(p, c.pending)
		c.pending = c.pending[n:]
		return n, nil
	}
	if c.readErr != nil {
		return 0, c.readErr
	}

	select {
	case res := <-c.readCh:
		n := copy(p, res.data)
		if n < len(res.data) {
			c.pending = res.data[n:]
		}
		if res.err != nil {
			c.readErr = res.err
			if n > 0 {
				return n, nil
			}
			return 0, res.err
		}
		return n, nil
	case <-c.readDeadline.wait():
		return 0, os.ErrDeadlineExceeded
	case <-c.closed:
		return 0, net.ErrClosed
	}
}

// Write sends p across the stream in chunks of at most MaxChunkSize.
func (c *StreamConn) Write(p []byte) (int, error) {
	c.writeMu.Lock()
	defer c.writeMu.Unlock()

	if isClosedChan(c.writeDeadline.wait()) {
		return 0, os.ErrDeadlineExceeded
	}
	if isClosedChan(c.closed) {
		return 0, net.ErrClosed
	}

	written := 0
	for len(p) > 0 {
		if isClosedChan(c.writeDeadline.wait()) {
			return written, os.ErrDeadlineExceeded
		}
		chunk := p
		if len(chunk) > MaxChunkSize {
			chunk = chunk[:MaxChunkSize]
		}
		if err := c.fns.Send(chunk); err != nil {
			return written, err
		}
		written += len(chunk)
		p = p[len(chunk):]
	}
	return written, nil
}

// SendKeepAlive sends an empty frame to keep intermediate proxies active.
func (c *StreamConn) SendKeepAlive() error {
	c.writeMu.Lock()
	defer c.writeMu.Unlock()
	if isClosedChan(c.closed) {
		return net.ErrClosed
	}
	return c.fns.Send(nil)
}

// CloseWrite half-closes the write side of the stream when supported, used by
// httputil.ReverseProxy on upgraded 101 connections.
func (c *StreamConn) CloseWrite() error {
	c.writeMu.Lock()
	defer c.writeMu.Unlock()
	c.closeWriteOnce.Do(func() {
		if c.fns.CloseWrite != nil {
			c.closeWriteErr = c.fns.CloseWrite()
		}
	})
	return c.closeWriteErr
}

// Close closes the connection and invokes CloseWrite and Close hooks once.
func (c *StreamConn) Close() error {
	c.closeOnce.Do(func() {
		close(c.closed)
		_ = c.CloseWrite()
		if c.fns.Close != nil {
			c.closeErr = c.fns.Close()
		}
	})
	return c.closeErr
}

// LocalAddr returns a synthetic local address for the stream.
func (c *StreamConn) LocalAddr() net.Addr { return streamAddr{} }

// RemoteAddr returns a synthetic remote address for the stream.
func (c *StreamConn) RemoteAddr() net.Addr { return streamAddr{} }

// SetDeadline sets both read and write deadlines.
func (c *StreamConn) SetDeadline(t time.Time) error {
	c.readDeadline.set(t)
	c.writeDeadline.set(t)
	return nil
}

// SetReadDeadline sets the deadline for future and pending Read calls.
func (c *StreamConn) SetReadDeadline(t time.Time) error {
	c.readDeadline.set(t)
	return nil
}

// SetWriteDeadline sets the deadline for future Write calls.
func (c *StreamConn) SetWriteDeadline(t time.Time) error {
	c.writeDeadline.set(t)
	return nil
}

type streamAddr struct{}

func (streamAddr) Network() string { return "grpc-relay" }
func (streamAddr) String() string  { return "grpc-relay" }

type deadline struct {
	mu     sync.Mutex
	timer  *time.Timer
	cancel chan struct{}
}

func newDeadline() deadline {
	return deadline{cancel: make(chan struct{})}
}

func (d *deadline) set(t time.Time) {
	d.mu.Lock()
	defer d.mu.Unlock()

	if d.timer != nil && !d.timer.Stop() {
		<-d.cancel
	}
	d.timer = nil

	closed := isClosedChan(d.cancel)
	if t.IsZero() {
		if closed {
			d.cancel = make(chan struct{})
		}
		return
	}
	if dur := time.Until(t); dur > 0 {
		if closed {
			d.cancel = make(chan struct{})
		}
		d.timer = time.AfterFunc(dur, func() {
			close(d.cancel)
		})
		return
	}
	if !closed {
		close(d.cancel)
	}
}

func (d *deadline) wait() chan struct{} {
	d.mu.Lock()
	defer d.mu.Unlock()
	return d.cancel
}

func isClosedChan(ch <-chan struct{}) bool {
	select {
	case <-ch:
		return true
	default:
		return false
	}
}
