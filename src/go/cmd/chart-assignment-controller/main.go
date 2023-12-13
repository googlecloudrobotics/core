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
	"log/slog"
	"net/http"
	"os"

	"contrib.go.opencensus.io/exporter/stackdriver"
	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"github.com/googlecloudrobotics/core/src/go/pkg/controller/chartassignment"
	"github.com/googlecloudrobotics/ilog"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/client-go/kubernetes/scheme"
	"k8s.io/client-go/rest"
	ctrllog "sigs.k8s.io/controller-runtime/pkg/log"
	"sigs.k8s.io/controller-runtime/pkg/log/zap"
	"sigs.k8s.io/controller-runtime/pkg/manager"
	"sigs.k8s.io/controller-runtime/pkg/manager/signals"
	metricsserver "sigs.k8s.io/controller-runtime/pkg/metrics/server"
	"sigs.k8s.io/controller-runtime/pkg/webhook"
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
	slog.SetDefault(slog.New(slog.NewJSONHandler(os.Stdout, nil)))

	ctx := context.Background()
	if *stackdriverProjectID != "" && *cloudCluster == false {
		sd, err := stackdriver.NewExporter(stackdriver.Options{
			ProjectID: *stackdriverProjectID,
		})
		if err != nil {
			slog.Error("Failed to create the Stackdriver exporter", ilog.Err(err))
			os.Exit(1)
		}
		trace.RegisterExporter(sd)
		trace.ApplyConfig(trace.Config{DefaultSampler: trace.AlwaysSample()})
		defer sd.Flush()
	}

	var clusterName string
	if *cloudCluster == true {
		clusterName = "cloud"
		slog.Info("Starting chart-assignment-controller in cloud setup")
	} else {
		clusterName = os.Getenv("ROBOT_NAME")
		slog.Info("Starting chart-assigment-controller in robot setup", slog.String("Cluster", clusterName))
		if clusterName == "" {
			slog.Error("expect ROBOT_NAME environment var to be set to an non-empty string")
			os.Exit(1)
		}
	}

	config, err := rest.InClusterConfig()
	if err != nil {
		slog.Error("Failed to load config", ilog.Err(err))
		os.Exit(1)
	}
	config.QPS = float32(*maxQPS)
	// The default value of twice the max QPS seems to work well.
	config.Burst = *maxQPS * 2

	slog.Error("Controller terminated", ilog.Err(runController(ctx, config, clusterName)))
	os.Exit(1)
}

func runController(ctx context.Context, cfg *rest.Config, cluster string) error {
	ctrllog.SetLogger(zap.New())

	sc := runtime.NewScheme()
	scheme.AddToScheme(sc)
	apps.AddToScheme(sc)

	mgr, err := manager.New(cfg, manager.Options{
		Scheme:                 sc,
		WebhookServer:          webhook.NewServer(webhook.Options{CertDir: *certDir, Port: *webhookPort}),
		Metrics:                metricsserver.Options{BindAddress: "0"}, // disabled
		HealthProbeBindAddress: ":8080",
	})
	if err != nil {
		return errors.Wrap(err, "create controller manager")
	}
	if err := chartassignment.Add(ctx, mgr, *cloudCluster); err != nil {
		return errors.Wrap(err, "add ChartAssignment controller")
	}
	mgr.AddHealthzCheck("trivial", func(_ *http.Request) error { return nil })

	if *webhookEnabled {
		webhook := chartassignment.NewValidationWebhook(mgr)
		srv := mgr.GetWebhookServer()
		srv.Register("/chartassignment/validate", webhook)
	}

	return mgr.Start(signals.SetupSignalHandler())
}
