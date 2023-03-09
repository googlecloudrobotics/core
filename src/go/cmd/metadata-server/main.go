// Copyright 2019 The Cloud Robotics Authors
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

// Package main runs a local http server providing details about the connected
// cloud project.
//
// This metadata server replicates a subset of the GKE metadata server
// functionality to provide applcation default creadentials for local services.
package main

import (
	"context"
	"flag"
	"fmt"
	"log"
	"net"
	"net/http"
	"os"
	"os/exec"
	"os/signal"
	"strconv"
	"syscall"

	"k8s.io/client-go/kubernetes"
	"k8s.io/client-go/rest"

	"github.com/fsnotify/fsnotify"
)

var (
	bindIP      = flag.String("bind_ip", "127.0.0.1", "IPv4 address to listen on")
	port        = flag.Int("port", 80, "Port number to listen on")
	robotIdFile = flag.String("robot_id_file", "", "robot-id.json file")
	sourceCidr  = flag.String("source_cidr", "127.0.0.1/32", "CIDR giving allowed source addresses for token retrieval")
	// Mirror gke behavior and return token with at least 5 minutes of remaining time.
	// https://cloud.google.com/compute/docs/access/create-enable-service-accounts-for-instances#applications
	minTokenExpiry = flag.Int("min_token_expiry", 300, "Minimum time a token needs to be valid for in seconds")
	logPeerDetails = flag.Bool("log_peer_details", false, "When enabled details about the peer that requests ADC are logged on the expense of some extra latency")
)

func detectChangesToFile(filename string) <-chan struct{} {
	watcher, err := fsnotify.NewWatcher()
	if err != nil {
		log.Fatal(err)
	}
	err = watcher.Add(filename)
	if err != nil {
		log.Fatal(err)
	}
	changes := make(chan struct{})
	go func() {
		for {
			select {
			case event, ok := <-watcher.Events:
				if !ok {
					return
				}
				log.Printf("event on %s: %s", filename, event)
				if event.Op&(fsnotify.Write|fsnotify.Remove) != 0 {
					changes <- struct{}{}
				}
			case err, ok := <-watcher.Errors:
				if !ok {
					return
				}
				log.Println("watcher error:", err)
			}
		}
	}()
	return changes
}

func addIPTablesRule() error {
	args := append([]string{"-t", "nat", "-A", "PREROUTING"}, getIPTablesRuleSpec(*port)...)
	if err := runIPTablesCommand(args); err != nil {
		return fmt.Errorf("iptables invocation failed: %v", err)
	}
	return nil
}

func removeIPTablesRule() {
	args := append([]string{"-t", "nat", "-D", "PREROUTING"}, getIPTablesRuleSpec(*port)...)
	if err := runIPTablesCommand(args); err != nil {
		log.Printf("Warning: iptables invocation failed: %v", err)
	}
}

func getIPTablesRuleSpec(port int) []string {
	return []string{
		"-p", "tcp",
		"-d", "169.254.169.254",
		"--dport", "80",
		"-j", "DNAT",
		"--to-destination", *bindIP + ":" + strconv.Itoa(port),
	}
}

func runIPTablesCommand(args []string) error {
	log.Printf("Debug: Running iptables command with args %v", args)
	cmd := exec.Command("iptables", args...)
	cmd.Stdout = os.Stdout
	cmd.Stderr = os.Stderr
	return cmd.Run()
}

func main() {
	flag.Parse()

	if ip := net.ParseIP(*bindIP); ip == nil {
		log.Fatal("invalid bind_ip flag")
	}

	ctx := context.Background()

	config, err := rest.InClusterConfig()
	if err != nil {
		log.Fatalf("Can't create k8s in-cluster config: %v", err)
	}
	k8s, err := kubernetes.NewForConfig(config)
	if err != nil {
		log.Fatalf("Can't create k8s client: %v", err)
	}

	tokenHandler, err := NewTokenHandler(ctx, k8s)
	if err != nil {
		log.Fatalf("%v", err)
	}

	http.Handle("/computeMetadata/v1/instance/service-accounts/default/token", tokenHandler)
	serviceAccountHandler := ServiceAccountHandler{}
	http.Handle("/computeMetadata/v1/instance/service-accounts/default/", serviceAccountHandler)
	http.Handle("/computeMetadata/v1/instance/service-accounts/", ConstHandler{[]byte("default/\n")})
	metadataHandler := tokenHandler.NewMetadataHandler(ctx)
	http.Handle("/computeMetadata/v1/", metadataHandler)
	http.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path == "/" {
			log.Printf("Handling root request for %s", r.URL.Path)
			w.Header().Set("Metadata-Flavor", "Google")
			w.Header().Set("Content-Type", "application/text")
			w.WriteHeader(http.StatusOK)
			w.Write([]byte("computeMetadata/\n"))
		} else {
			log.Printf("Unhandled request %s from %s", r.URL.Path, r.RemoteAddr)
			http.NotFound(w, r)
		}
	})
	// Return dummy IP addresses for internal/external IPs. cloudprober
	// crashes if these are not present.
	http.Handle("/computeMetadata/v1/instance/network-interfaces/0/ip", ConstHandler{[]byte("127.0.0.1/\n")})
	http.Handle("/computeMetadata/v1/instance/network-interfaces/0/access-configs/0/external-ip", ConstHandler{[]byte("127.0.0.1/\n")})

	// Make sure that the bind is successful before adding the iptables rule as a means of
	// avoiding a race condition where an old metadata-server instance that has not yet fully
	// terminated removes the iptables rule again.
	bindAddress := fmt.Sprintf("%s:%d", *bindIP, *port)
	ln, err := net.Listen("tcp", bindAddress)
	if err != nil {
		log.Fatalf("failed to create listener on %s: %v", bindAddress, err)
	}
	log.Printf("Listening on %s", bindAddress)

	if err := addIPTablesRule(); err != nil {
		log.Fatalf("failed to add iptables rule: %v", err)
	}

	if err := PatchCorefile(ctx, k8s); err != nil {
		removeIPTablesRule()
		log.Fatal(err)
	}

	go func() {
		err = http.Serve(ln, nil)
		RevertCorefile(ctx, k8s)
		removeIPTablesRule()
		log.Fatal(err)
	}()

	go func() {
		<-detectChangesToFile(*robotIdFile)
		RevertCorefile(ctx, k8s)
		removeIPTablesRule()
		log.Fatalf("%s changed but reloading is not implemented. Crashing...", *robotIdFile)
	}()

	stop := make(chan os.Signal)
	signal.Notify(stop, syscall.SIGTERM)

	<-stop
	RevertCorefile(ctx, k8s)
	removeIPTablesRule()
}
