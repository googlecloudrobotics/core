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
	"context"
	"flag"
	"log/slog"
	"os"
	"time"

	"github.com/googlecloudrobotics/core/src/go/pkg/gcr"
	"github.com/googlecloudrobotics/core/src/go/pkg/robotauth"
	"github.com/googlecloudrobotics/ilog"
	"k8s.io/client-go/kubernetes"
	"k8s.io/client-go/rest"
)

var (
	robotIdFile = flag.String("robot_id_file", "", "robot-id.json file")
)

const updateInterval = 10 * time.Minute

// Updates the token used to pull images from GCR in the surrounding cluster.
func updateCredentials(ctx context.Context) error {
	// Connect to the surrounding k8s cluster.
	localConfig, err := rest.InClusterConfig()
	if err != nil {
		slog.Error("InClusterConfig", ilog.Err(err))
		os.Exit(1)
	}
	localClient, err := kubernetes.NewForConfig(localConfig)
	if err != nil {
		slog.Error("NewForConfig", ilog.Err(err))
		os.Exit(1)
	}
	robotAuth, err := robotauth.LoadFromFile(*robotIdFile)
	if err != nil {
		slog.Error("failed to read robot id file",
			slog.String("File", *robotIdFile),
			ilog.Err(err))
		os.Exit(1)
	}
	// Perform a token exchange with the TokenVendor in the cloud cluster and update the
	// credentials used to pull images from GCR.
	return gcr.UpdateGcrCredentials(ctx, localClient, robotAuth)
}

// Updates the token used to pull images from GCR in the surrounding cluster. The update runs
// on startup, and then every 10 minutes.
func main() {
	flag.Parse()
	logHandler := ilog.NewLogHandler(slog.LevelInfo, os.Stderr)
	slog.SetDefault(slog.New(logHandler))
	ctx := context.Background()

	for {
		if err := updateCredentials(ctx); err != nil {
			slog.Error("Update Credentials", ilog.Err(err))
			os.Exit(1)
		}
		slog.Info("Updated GCR credentials in local cluster")
		time.Sleep(updateInterval)
	}
}
