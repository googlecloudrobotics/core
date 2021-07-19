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

package main

import (
	"flag"
	"log"
	"time"

	"github.com/googlecloudrobotics/core/src/go/pkg/gcr"
	"github.com/googlecloudrobotics/core/src/go/pkg/robotauth"
	"k8s.io/client-go/kubernetes"
	"k8s.io/client-go/rest"
)

var (
	robotIdFile = flag.String("robot_id_file", "", "robot-id.json file")
)

const updateInterval = 10 * time.Minute

// Updates the token used to pull images from GCR in the surrounding cluster.
func updateCredentials() error {
	// Connect to the surrounding k8s cluster.
	localConfig, err := rest.InClusterConfig()
	if err != nil {
		log.Fatal(err)
	}
	localClient, err := kubernetes.NewForConfig(localConfig)
	if err != nil {
		log.Fatal(err)
	}
	robotAuth, err := robotauth.LoadFromFile(*robotIdFile)
	if err != nil {
		log.Fatalf("failed to read robot id file %s: %v", *robotIdFile, err)
	}
	// Perform a token exchange with the TokenVendor in the cloud cluster and update the
	// credentials used to pull images from GCR.
	return gcr.UpdateGcrCredentials(localClient, robotAuth)
}

// Updates the token used to pull images from GCR in the surrounding cluster. The update runs
// on startup, and then every 10 minutes.
func main() {
	flag.Parse()

	for {
		if err := updateCredentials(); err != nil {
			log.Fatal(err)
		}
		log.Printf("Updated GCR credentials in local cluster")
		time.Sleep(updateInterval)
	}
}
