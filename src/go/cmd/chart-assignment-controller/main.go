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

// Package main defines the entry point for the chart assignment controller service.
//
// Ensures selected apps are running on the robot.
package main

import (
	"context"
	"flag"
	"log"
	"net/http"
	"os"

	"contrib.go.opencensus.io/exporter/stackdriver"
	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"github.com/googlecloudrobotics/core/src/go/pkg/controller/chartassignment"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/client-go/kubernetes/scheme"
	"k8s.io/client-go/rest"
	ctrllog "sigs.k8s.io/controller-runtime/pkg/log"
	"sigs.k8s.io/controller-runtime/pkg/log/zap"
	"sigs.k8s.io/controller-runtime/pkg/manager"
	"sigs.k8s.io/controller-runtime/pkg/manager/signals"
	"sigs.k8s.io/controller-runtime/pkg/webhook/admission"
)

var (
	cloudCluster = flag.Bool("cloud-cluster", true,
		"Is the controller deployed in cloud cluster")

	webhookEnabled = flag.Bool("webhook-enabled", true,
		"Whether the webhook should be served")

	webhookPort = flag.Int("webhook-port", 9876,
		"Listening port of the custom resource webhook")

	certDir = flag.String("cert-dir", "",
		"Directory for TLS certificates")

	stackdriverProjectID = flag.String("trace-stackdriver-project-id", "",
		"If not empty, traces will be uploaded to this Google Cloud Project. Not relevant for cloud cluster")

	maxQPS = flag.Int("apiserver-max-qps", 50,
		"Maximum number of calls to the API server per second.")
)

func main() {
	flag.Parse()
	ctx := context.Background()
	if *stackdriverProjectID != "" && *cloudCluster == false {
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

	var clusterName string
	if *cloudCluster == true {
		clusterName = "cloud"
		log.Print("Starting chart-assigment-controller in cloud setup")
	} else {
		clusterName = os.Getenv("ROBOT_NAME")
		log.Printf("Starting chart-assigment-controller in robot setup with cluster name %s", clusterName)
		if clusterName == "" {
			log.Fatalf("expect ROBOT_NAME environment var to be set to an non-empty string")
		}
	}

	config, err := rest.InClusterConfig()
	if err != nil {
		log.Fatalln(err)
	}
	config.QPS = float32(*maxQPS)
	// The default value of twice the max QPS seems to work well.
	config.Burst = *maxQPS * 2

	if err := setupAppV2(ctx, config, clusterName); err != nil {
		log.Fatalln(err)
	}

	// Run a k8s liveness probe in the main thread.
	http.HandleFunc("/healthz", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "text/plain")
		w.Write([]byte("ok"))
	})
	log.Fatal(http.ListenAndServe(":8080", nil))
}

func setupAppV2(ctx context.Context, cfg *rest.Config, cluster string) error {
	ctrllog.SetLogger(zap.New())

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
	if err := chartassignment.Add(ctx, mgr, cluster); err != nil {
		return errors.Wrap(err, "add ChartAssignment controller")
	}

	if *webhookEnabled {
		var webhook *admission.Webhook
		if *cloudCluster {
			webhook = chartassignment.NewValidationWebhook(mgr)
		} else {
			webhook = chartassignment.NewValidationWebhookForEdgeCluster(mgr, cluster)
		}
		srv := mgr.GetWebhookServer()
		srv.CertDir = *certDir
		srv.Register("/chartassignment/validate", webhook)
	}

	go func() {
		if err := mgr.Start(signals.SetupSignalHandler()); err != nil {
			log.Fatal(errors.Wrap(err, "start controller manager"))
		}
	}()
	return nil
}
