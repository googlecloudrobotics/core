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

// Package main defines the entry point for the robot master service.
//
// Ensures selected apps are running on the robot.
package main

import (
	"flag"
	"fmt"
	"log"
	"net/http"
	"os"
	"time"

	pb "src/proto/registry"

	"cloud-robotics.googlesource.com/cloud-robotics/pkg/helm"
	"golang.org/x/net/context"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/client-go/dynamic"
	"k8s.io/client-go/rest"
)

const (
	errBackoffInterval = 10 * time.Second
	pollInterval       = 30 * time.Second
)

func syncAppStateOnce(ctx context.Context, robotClient dynamic.ResourceInterface, h *helm.Helm, robotName string) error {
	robot, err := robotClient.Get(robotName, metav1.GetOptions{})
	if err != nil {
		return fmt.Errorf("failed to get Robot '%s': %v", robotName, err)
	}
	if err := h.InstallApps(ctx, "cloud-robotics-apps", pb.InstallationTarget_ROBOT, []unstructured.Unstructured{*robot}); err != nil {
		return fmt.Errorf("failed to update robot apps for '%s': %v", robotName, err)
	}
	return nil
}

func syncAppStateTask(ctx context.Context, k8sClient dynamic.Interface, h *helm.Helm, robotName string) {
	tick := time.NewTicker(pollInterval)
	defer tick.Stop()
	robotGVR := schema.GroupVersionResource{Group: "registry.cloudrobotics.com", Version: "v1alpha1", Resource: "robots"}
	robotClient := k8sClient.Resource(robotGVR).Namespace("default")

	for {
		if err := syncAppStateOnce(ctx, robotClient, h, robotName); err != nil {
			log.Println(err)
			time.Sleep(errBackoffInterval)
		}
		select {
		case <-h.UpdateChannel():
			log.Printf("Update triggered by Kubernetes change")
		case <-tick.C:
		}
	}
}

func main() {
	flag.Parse()
	ctx := context.Background()

	robotName := os.Getenv("ROBOT_NAME")
	if robotName == "" {
		panic(fmt.Errorf("expect ROBOT_NAME environment var to be set to an non-empty string"))
	}

	config, err := rest.InClusterConfig()
	if err != nil {
		panic(err)
	}
	k8sClient, err := dynamic.NewForConfig(config)
	if err != nil {
		panic(err)
	}

	h, err := helm.NewHelm(k8sClient)
	if err != nil {
		panic(err.Error())
	}

	go syncAppStateTask(ctx, k8sClient, h, robotName)

	// Run a k8s liveness probe in the main thread.
	http.HandleFunc("/healthz", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "text/plain")
		w.Write([]byte("ok"))
	})
	log.Fatal(http.ListenAndServe(":8080", nil))
}
