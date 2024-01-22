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

// Runs the app rollout controller which creates and deletes Kubernetes deployments
// to bring them into agreement with configuration.
package main

import (
	"context"
	"flag"
	"fmt"
	"log/slog"
	"os"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	registry "github.com/googlecloudrobotics/core/src/go/pkg/apis/registry/v1alpha1"
	"github.com/googlecloudrobotics/core/src/go/pkg/controller/approllout"
	"github.com/googlecloudrobotics/ilog"
	"github.com/pkg/errors"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/client-go/kubernetes/scheme"
	"k8s.io/client-go/rest"
	"k8s.io/helm/pkg/chartutil"
	"k8s.io/helm/pkg/strvals"
	"sigs.k8s.io/controller-runtime/pkg/healthz"
	ctrllog "sigs.k8s.io/controller-runtime/pkg/log"
	"sigs.k8s.io/controller-runtime/pkg/log/zap"
	"sigs.k8s.io/controller-runtime/pkg/manager"
	"sigs.k8s.io/controller-runtime/pkg/manager/signals"
	metricsserver "sigs.k8s.io/controller-runtime/pkg/metrics/server"
	"sigs.k8s.io/controller-runtime/pkg/webhook"
)

var (
	params = flag.String("params", "",
		"Helm configuration parameters formatted as name=value,topname.subname=value")

	healthzPort = flag.Int("healthz-port", 8080,
		"Listening port of the /healthz probe")

	webhookPort = flag.Int("webhook-port", 9876,
		"Listening port of the custom resource webhook")

	certDir = flag.String("cert-dir", "",
		"Directory for TLS certificates")
)

func main() {
	flag.Parse()
	logHandler := ilog.NewLogHandler(slog.LevelInfo, os.Stderr)
	slog.SetDefault(slog.New(logHandler))

	ctx := context.Background()
	kubernetesConfig, err := rest.InClusterConfig()
	if err != nil {
		slog.Error("Failed to initialize Kubernetes config", ilog.Err(err))
		os.Exit(1)
	}

	helmParams, err := strvals.ParseString(*params)
	if err != nil {
		slog.Error("invalid Helm parameters", ilog.Err(err))
		os.Exit(1)
	}

	if err := runController(ctx, kubernetesConfig, helmParams); err != nil {
		slog.Error("Exit", ilog.Err(runController(ctx, kubernetesConfig, helmParams)))
		os.Exit(1)
	}
	slog.Info("Exit")
}

func runController(ctx context.Context, cfg *rest.Config, params map[string]interface{}) error {
	ctrllog.SetLogger(zap.New())

	sc := runtime.NewScheme()
	scheme.AddToScheme(sc)
	apps.AddToScheme(sc)
	registry.AddToScheme(sc)

	mgr, err := manager.New(cfg, manager.Options{
		Scheme:                 sc,
		WebhookServer:          webhook.NewServer(webhook.Options{CertDir: *certDir, Port: *webhookPort}),
		Metrics:                metricsserver.Options{BindAddress: "0"}, // disabled
		HealthProbeBindAddress: fmt.Sprintf(":%d", *healthzPort),
	})
	if err != nil {
		return errors.Wrap(err, "create controller manager")
	}
	if err := approllout.Add(ctx, mgr, chartutil.Values(params)); err != nil {
		return errors.Wrap(err, "add AppRollout controller")
	}
	if err := mgr.AddHealthzCheck("trivial", healthz.Ping); err != nil {
		return errors.Wrap(err, "add healthz check")
	}

	srv := mgr.GetWebhookServer()

	srv.Register("/approllout/validate", approllout.NewAppRolloutValidationWebhook(mgr))
	srv.Register("/app/validate", approllout.NewAppValidationWebhook(mgr))

	return mgr.Start(signals.SetupSignalHandler())
}
