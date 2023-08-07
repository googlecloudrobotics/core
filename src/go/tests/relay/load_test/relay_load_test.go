package main

import (
	"fmt"
	"html"
	"io"
	"log"
	"net/http"
	"time"

	"github.com/googlecloudrobotics/core/src/go/cmd/http-relay-client/client"
	"github.com/googlecloudrobotics/core/src/go/cmd/http-relay-server/server"
)

const (
	port = 1976
)

var myClient *http.Client

func startWebserver() {
	http.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		time.Sleep(time.Millisecond * 50)
		fmt.Fprintf(w, "Hello, %q", html.EscapeString(r.URL.Path))
	})
	go http.ListenAndServe(fmt.Sprint(":", port), nil)
}

func startLoadTest(serverName string, port int) {
	count := 0
	for {
		request, err := http.NewRequest("GET", fmt.Sprint("http://localhost:", port), nil)
		if serverName != "" {
			request.URL.Path = fmt.Sprint("/client/", serverName)
		}
		if err != nil {
			panic(fmt.Sprintf("Got error: %v", err))
		}
		resp, err := myClient.Do(request)
		if err != nil {
			panic(fmt.Sprintf("Got error: %v", err))
		}
		defer resp.Body.Close()
		body, err := io.ReadAll(resp.Body)
		if err != nil {
			fmt.Println(err)
		}
		log.Printf("Finished GET request #%v with body: %+v", count, string(body))
		count += 1
	}
}

func main() {
	// Customize the Transport to have larger connection pool
	transport := http.DefaultTransport.(*http.Transport).Clone()
	transport.MaxIdleConns = 200
	transport.MaxIdleConnsPerHost = 200

	myClient = &http.Client{Transport: transport}

	relay_server := server.NewServer()

	client_config := client.DefaultClientConfig()
	client_config.ServerName = "vm-load-test"
	client_config.RelayAddress = "localhost:10976"
	client_config.RelayScheme = "http"
	client_config.BackendAddress = fmt.Sprint("localhost:", port)
	client_config.BackendScheme = "http"
	client_config.MaxIdleConnsPerHost = 100
	relay_client := client.NewClient(client_config)

	go func() {
		relay_server.Start(10976, client_config.BlockSize)
	}()
	time.Sleep(2 * time.Second)
	go func() {
		relay_client.Start()
	}()
	time.Sleep(2 * time.Second)

	// start a webserver in a goroutine
	startWebserver()

	log.Println("Load test is tarting.")

	for i := 0; i < 100; i++ {
		// go startLoadTest("", 1976)
		go startLoadTest(client_config.ServerName, 10976)
	}

	time.Sleep(time.Second * 2400)
}
