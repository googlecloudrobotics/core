// Copyright 2019 The Google Cloud Robotics Authors
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

// Runs the cloud master which creates and deletes Kubernetes deployments
// to bring them into agreement with configuration.
package main

import (
	"context"
	"flag"
	"log"
	"net/http"
	"os"
	"time"

	"k8s.io/client-go/rest"
)

const (
	numRetries         = 3
	errBackoffInterval = 10 * time.Second
	pollInterval       = 10 * time.Second
)

var (
	kubeconfig = flag.String("kubeconfig", "",
		"path to the kubeconfig file, $HOME/.kube/config for out-of-cluster")
)

func reportHealth(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "text/plain")
	w.Write([]byte("ok\n"))
}

func runHealthCheckServer() {
	http.HandleFunc("/healthz", reportHealth)
	log.Fatal(http.ListenAndServe(":8080", nil))
}

func spinOnce(ctx context.Context, cm *cloudMaster, sp *statusPublisher) error {
	if err := cm.spin(ctx); err != nil {
		return err
	}
	return sp.spin()
}

// spin runs the update cycle periodically. It returns if a fixed number of
// updates have failed consecutively, otherwise it continues indefinitely.
func spin(ctx context.Context, cm *cloudMaster, sp *statusPublisher) {
	tick := time.NewTicker(pollInterval)
	defer tick.Stop()
	numFailures := 0
	for numFailures < numRetries {
		if err := spinOnce(ctx, cm, sp); err != nil {
			log.Printf("Cloud master iteration failed: %v", err)
			numFailures += 1
			time.Sleep(errBackoffInterval)
		} else {
			numFailures = 0
		}
		select {
		case <-cm.h.UpdateChannel():
			log.Printf("Update triggered by Kubernetes change")
		case <-tick.C:
		}
	}
}

func main() {
	log.SetFlags(log.Lshortfile | log.LstdFlags)
	flag.Parse()

	ctx := context.Background()

	kubernetesConfig, err := rest.InClusterConfig()
	if err != nil {
		log.Fatalf("Failed to initialize Kubernetes config: %v", err)
	}

	cm, err := newCloudMaster(kubernetesConfig)
	if err != nil {
		log.Fatalf("Unable to create cloud master: %v", err)
	}

	sp, err := newStatusPublisher(os.Getenv("GOOGLE_CLOUD_PROJECT"), kubernetesConfig)
	if err != nil {
		log.Fatalf("Unable to start status publisher: %v", err)
	}

	go runHealthCheckServer()
	spin(ctx, cm, sp)
	log.Fatalf("Cloud master terminating due to repeated errors")
}
