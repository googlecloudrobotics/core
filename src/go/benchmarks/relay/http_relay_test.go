package main

import (
	"fmt"
	"io"
	"net"
	"net/http"
	"net/http/httptest"
	"os/exec"
	"syscall"
	"testing"
	"time"

	"github.com/bazelbuild/rules_go/go/runfiles"
	"github.com/cenkalti/backoff"
)

type TestCase struct {
	payloadSize int
	blockSize   int
}

type TestFixture struct {
	backendUrl  string
	relayUrl    string
	stopServers func()
}

func newTestFixture(b *testing.B, tc TestCase) TestFixture {
	payload := make([]byte, tc.payloadSize)
	ts := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path == "/infinite" {
			for {
				if _, err := w.Write(payload); err != nil {
					return
				}
			}
		}

		w.Write(payload)
	}))

	relayServerBin, err := runfiles.Rlocation("cloud_robotics/src/go/cmd/http-relay-server/http-relay-server-app_/http-relay-server-app")
	if err != nil {
		b.Fatal(err)
	}
	relayPort := 8000
	relayServer := exec.Command(relayServerBin, fmt.Sprintf("-port=%d", relayPort), fmt.Sprintf("-block_size=%d", tc.blockSize))
	if err := relayServer.Start(); err != nil {
		b.Fatal(err)
	}
	relayUrl := fmt.Sprint("http://localhost:", relayPort, "/client/test-server/")

	relayClientBin, err := runfiles.Rlocation("cloud_robotics/src/go/cmd/http-relay-client/http-relay-client-app_/http-relay-client-app")
	if err != nil {
		b.Fatal(err)
	}
	relayClient := exec.Command(relayClientBin,
		"-server_name=test-server",
		"-disable_auth_for_remote",
		fmt.Sprintf("-relay_address=localhost:%d", relayPort),
		fmt.Sprintf("-relay_scheme=http"),
		fmt.Sprintf("-backend_address=%s", ts.Listener.Addr().(*net.TCPAddr).String()),
		fmt.Sprintf("-backend_scheme=http"),
		fmt.Sprintf("-block_size=%d", tc.blockSize))
	if err := relayClient.Start(); err != nil {
		b.Fatal(err)
	}

	// wait for the relays to be ready
	cbo := backoff.NewConstantBackOff(1 * time.Second)
	bo := backoff.WithMaxRetries(cbo, 10)
	if err := backoff.Retry(func() error {
		resp, err := http.Get(relayUrl)
		if err != nil {
			return err
		}
		resp.Body.Close()
		return nil
	}, bo); err != nil {
		b.Fatal(err)
	}

	stopServers := func() {
		ts.CloseClientConnections()
		ts.Close()
		relayServer.Process.Signal(syscall.SIGTERM)
		relayClient.Process.Signal(syscall.SIGTERM)
		relayServer.Wait()
		relayClient.Wait()
	}

	return TestFixture{backendUrl: ts.URL, relayUrl: relayUrl, stopServers: stopServers}
}

func benchmarkHttpRelay(b *testing.B, url string) {
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		resp, err := http.Get(url)
		if err != nil {
			b.Fatal(err)
		}
		defer resp.Body.Close()
		_, err = io.ReadAll(resp.Body)
		if err != nil {
			b.Fatal(err)
		}
	}
	b.StopTimer()
}

func benchmarkHttpRelaySingleSlowClient(b *testing.B, url string) {
	resp, err := http.Get(fmt.Sprintf("%s/infinite", url))
	if err != nil {
		b.Fatal(err)
	}
	defer resp.Body.Close()
	stop := make(chan struct{})
	done := make(chan struct{})
	go func() {
		for {
			select {
			case <-stop:
				close(done)
				return
			default:
				_, err := resp.Body.Read(make([]byte, 1024))
				if err != nil {
					b.Fatal(err)
				}
			}
			time.Sleep(10 * time.Millisecond)
		}
	}()
	// let the buffer fill
	time.Sleep(1)

	benchmarkHttpRelay(b, url)

	close(stop)
	<-done
}

func BenchmarkHttpRelay(b *testing.B) {
	tests := []struct {
		payloadSize int
		blockSize   int
	}{
		{payloadSize: 1024, blockSize: 10 * 1024},
		// {payloadSize: 10 * 1024, blockSize: 10 * 1024},
		// {payloadSize: 100 * 1024, blockSize: 10 * 1024},
		// {payloadSize: 1024 * 1024, blockSize: 10 * 1024},
	}

	for _, tc := range tests {
		tf := newTestFixture(b, tc)
		b.Run(fmt.Sprintf("BenchmarkHttpRelayDirect_Payload%d", tc.payloadSize), func(b *testing.B) {
			benchmarkHttpRelay(b, tf.backendUrl)
		})
		b.Run(fmt.Sprintf("BenchmarkHttpRelay_Payload%d", tc.payloadSize), func(b *testing.B) {
			benchmarkHttpRelay(b, tf.relayUrl)
		})
		b.Run(fmt.Sprintf("BenchmarkHttpRelaySingleSlowClient_Payload%d", tc.payloadSize), func(b *testing.B) {
			benchmarkHttpRelaySingleSlowClient(b, tf.relayUrl)
		})
		tf.stopServers()
	}
}
