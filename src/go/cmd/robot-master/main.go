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

// Package main defines the entry point for the robot master service.
//
// Ensures selected apps are running on the robot.
package main

import (
	"flag"
	"log"
	"net/http"
	"os"
	"strings"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"github.com/googlecloudrobotics/core/src/go/pkg/controller/chartassignment"
	"github.com/pkg/errors"
	"go.opencensus.io/exporter/stackdriver"
	"go.opencensus.io/trace"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/client-go/kubernetes/scheme"
	"k8s.io/client-go/rest"
	"sigs.k8s.io/controller-runtime/pkg/manager"
	ctrllog "sigs.k8s.io/controller-runtime/pkg/runtime/log"
	"sigs.k8s.io/controller-runtime/pkg/runtime/signals"
)

var (
	webhookEnabled = flag.Bool("webhook-enabled", true,
		"Whether the webhook should be served")

	webhookPort = flag.Int("webhook-port", 9876,
		"Listening port of the custom resource webhook")

	certDir = flag.String("cert-dir", "",
		"Directory for TLS certificates")

	stackdriverProjectID = flag.String("trace-stackdriver-project-id", "",
		"If not empty, traces will be uploaded to this Google Cloud Project")
)

func main() {
	flag.Parse()
	if *stackdriverProjectID != "" {
		sd, err := stackdriver.NewExporter(stackdriver.Options{
			ProjectID: *stackdriverProjectID,
		})
		if err != nil {
			log.Fatalf("Failed to create the Stackdriver exporter: %v", err)
		}
		trace.RegisterExporter(sd)
		trace.ApplyConfig(trace.Config{DefaultSampler: trace.AlwaysSample()})
		defer sd.Flush()
	}

	robotName := os.Getenv("ROBOT_NAME")
	if robotName == "" {
		log.Fatalf("expect ROBOT_NAME environment var to be set to an non-empty string")
	}

	config, err := rest.InClusterConfig()
	if err != nil {
		log.Fatalln(err)
	}

	if err := setupAppV2(config, robotName); err != nil {
		log.Fatalln(err)
	}

	// Run a k8s liveness probe in the main thread.
	http.HandleFunc("/healthz", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "text/plain")
		w.Write([]byte("ok"))
	})
	log.Fatal(http.ListenAndServe(":8080", nil))
}

func setupAppV2(cfg *rest.Config, cluster string) error {
	ctrllog.SetLogger(ctrllog.ZapLogger(true))

	sc := runtime.NewScheme()
	scheme.AddToScheme(sc)
	apps.AddToScheme(sc)

	mgr, err := manager.New(cfg, manager.Options{
		Scheme:             sc,
		Port:               *webhookPort,
		MetricsBindAddress: "0", // disabled
	})
	if err != nil {
		return errors.Wrap(err, "create controller manager")
	}
	if err := chartassignment.Add(mgr, cluster); err != nil {
		return errors.Wrap(err, "add ChartAssignment controller")
	}
	if *webhookEnabled {
		srv := mgr.GetWebhookServer()
		srv.CertDir = *certDir

		webhook := chartassignment.NewValidationWebhookForEdgeCluster(mgr, cluster)
		srv.Register("/chartassignment/validate", webhook)
	}

	go func() {
		if err := mgr.Start(signals.SetupSignalHandler()); err != nil {
			log.Fatal(errors.Wrap(err, "start controller manager"))
		}
	}()
	return nil
}

func parseLabels(s string) (map[string]string, error) {
	lset := map[string]string{}

	for _, l := range strings.Split(s, ",") {
		parts := strings.SplitN(l, "=", 2)
		if len(parts) != 2 {
			return nil, errors.New("invalid labels")
		}
		lset[parts[0]] = parts[1]
	}
	return lset, nil
}
