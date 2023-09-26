package main

import (
	"io"
	"net/http"
	"sync"
	"testing"
	"time"

	"github.com/golang/glog"

	"github.com/googlecloudrobotics/core/src/go/cmd/http-relay-client/client"
	"github.com/googlecloudrobotics/core/src/go/cmd/http-relay-server/server"
)

var (
	port      = 8081
	blockSize = 10 * 1024
	once      sync.Once
)

func initRelay() {
	once.Do(func() {
		glog.Info("Running init.")

		go func() {
			relayServer := server.NewServer()
			relayServer.Start(port, blockSize)
		}()

		go func() {
			config := client.DefaultClientConfig()
			config.BackendScheme = "http"
			config.RelayScheme = "http"
			config.BackendAddress = "127.0.0.1:8083"
			relayClient := client.NewClient(config)
			relayClient.Start()
		}()

		relayHealthy := false
		deadline := time.Now().Add(5 * time.Second)
		for time.Now().Before(deadline) {
			res, err := http.Get("http://127.0.0.1:8081/healthz")
			if err != nil {
				glog.Infof("Relay server is has not yet started, retrying.")
				time.Sleep(250 * time.Millisecond)
			} else {
				glog.Info("Relay server is up and running.")
				relayHealthy = true
				defer res.Body.Close()
				io.ReadAll(res.Body)
				break
			}
		}
		if !relayHealthy {
			glog.Fatal("Failed to bring up http relay for unknown reason.")
		}
	})
}

func TestTimeout(t *testing.T) {
	initRelay()

	go func() {
		httpHandlerFunc := http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			// TBD: server code, e.g. sleeping to force a timeout
			glog.Info("reached server side endpoint")
			time.Sleep(6 * time.Second)
			http.DefaultServeMux.ServeHTTP(w, r)
		})
		srv := &http.Server{
			Addr:         "127.0.0.1:8083",
			ReadTimeout:  5 * time.Second,
			WriteTimeout: 5 * time.Second,
			Handler: http.TimeoutHandler(httpHandlerFunc,
				5*time.Second, "handler timeout"),
		}
		srv.ListenAndServe()
	}()

	glog.Info("Sending request to backend.")

	relayAddress := "http://127.0.0.1:8081/client/server_name/"
	res, err := http.Get(relayAddress)
	if err != nil {
		t.Fatal(err)
	}
	defer res.Body.Close()
	io.ReadAll(res.Body)
}
