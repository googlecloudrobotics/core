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
	"log"

	"github.com/googlecloudrobotics/core/src/go/pkg/gcr"
	"github.com/googlecloudrobotics/core/src/go/pkg/robotauth"
	k8serrors "k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/client-go/kubernetes"
	"k8s.io/client-go/rest"
)

// Updates the token used to pull images from GCR in the surrounding cluster. Exits after the update
// has finished and thus should be run as a cron job in order to prevent the token from expiring.
func main() {
	// Connect to the surrounding k8s cluster.
	localConfig, err := rest.InClusterConfig()
	if err != nil {
		log.Fatal(err)
	}
	localClient, err := kubernetes.NewForConfig(localConfig)
	if err != nil {
		log.Fatal(err)
	}
	// Read the robot auth data from the corresponding k8s secret.
	robotAuth, err := robotauth.LoadFromK8sSecret(localClient)
	if k8serrors.IsNotFound(err) {
		log.Printf("Warning: Robot auth secret not found. Not refreshing GCR " +
			"credentials... (this is only OK if the robot cluster is running " +
			"in the cloud).")
		return
	}
	if err != nil {
		log.Fatal(err)
	}
	// Perform a token exchange with the TokenVendor in the cloud cluster and update the
	// credentials used to pull images from GCR.
	if err := gcr.UpdateGcrCredentials(localClient, robotAuth); err != nil {
		log.Fatal(err)
	}
	log.Printf("Updated gcr credentials in local cluster")
}
