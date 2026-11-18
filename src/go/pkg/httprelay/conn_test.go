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

package httprelay

import (
	"bytes"
	"errors"
	"io"
	"os"
	"testing"
	"time"
)

func TestStreamConnReadAndKeepAlive(t *testing.T) {
	frames := [][]byte{
		{}, // keepalive frame, must be skipped
		[]byte(" world"),
	}
	idx := 0
	conn := NewStreamConn([]byte("hello"), StreamFuncs{
		Recv: func() ([]byte, error) {
			if idx >= len(frames) {
				return nil, io.EOF
			}
			f := frames[idx]
			idx++
			return f, nil
		},
	})
	defer conn.Close()

	got, err := io.ReadAll(conn)
	if err != nil {
		t.Fatalf("io.ReadAll(conn) returned unexpected error: %v", err)
	}
	if want := "hello world"; string(got) != want {
		t.Errorf("io.ReadAll(conn) = %q, want %q", got, want)
	}
}

func TestStreamConnWriteChunksLargePayloads(t *testing.T) {
	var chunks []int
	var combined bytes.Buffer
	conn := NewStreamConn(nil, StreamFuncs{
		Send: func(b []byte) error {
			chunks = append(chunks, len(b))
			combined.Write(b)
			return nil
		},
		Recv: func() ([]byte, error) {
			return nil, io.EOF
		},
	})
	defer conn.Close()

	payload := bytes.Repeat([]byte("a"), MaxChunkSize*2+17)
	n, err := conn.Write(payload)
	if err != nil {
		t.Fatalf("conn.Write(%d bytes) failed: %v", len(payload), err)
	}
	if n != len(payload) {
		t.Errorf("conn.Write(%d bytes) = %d, want %d", len(payload), n, len(payload))
	}
	if len(chunks) != 3 {
		t.Errorf("len(chunks) = %d, want 3", len(chunks))
	}
	if !bytes.Equal(combined.Bytes(), payload) {
		t.Errorf("combined payload length = %d, want %d matching bytes", combined.Len(), len(payload))
	}
}

func TestStreamConnReadDeadlineInterruptsWithoutLosingData(t *testing.T) {
	msgCh := make(chan []byte, 1)
	conn := NewStreamConn(nil, StreamFuncs{
		Recv: func() ([]byte, error) {
			b, ok := <-msgCh
			if !ok {
				return nil, io.EOF
			}
			return b, nil
		},
	})
	defer func() {
		close(msgCh)
		conn.Close()
	}()

	readErrCh := make(chan error, 1)
	go func() {
		buf := make([]byte, 16)
		_, err := conn.Read(buf)
		readErrCh <- err
	}()

	// Interrupt the blocked Read like net/http's abortPendingRead does.
	if err := conn.SetReadDeadline(time.Now().Add(-time.Second)); err != nil {
		t.Fatalf("conn.SetReadDeadline(past) failed: %v", err)
	}
	if err := <-readErrCh; !errors.Is(err, os.ErrDeadlineExceeded) {
		t.Fatalf("conn.Read() after past deadline = %v, want %v", err, os.ErrDeadlineExceeded)
	}

	// Clearing the deadline must allow subsequent data to be read intact.
	if err := conn.SetReadDeadline(time.Time{}); err != nil {
		t.Fatalf("conn.SetReadDeadline(zero) failed: %v", err)
	}
	msgCh <- []byte("after-deadline")
	buf := make([]byte, 32)
	n, err := conn.Read(buf)
	if err != nil {
		t.Fatalf("conn.Read() after clearing deadline failed: %v", err)
	}
	if got, want := string(buf[:n]), "after-deadline"; got != want {
		t.Errorf("conn.Read() = %q, want %q", got, want)
	}
}
