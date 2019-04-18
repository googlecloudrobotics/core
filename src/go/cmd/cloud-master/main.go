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

// Runs the cloud master which creates and deletes Kubernetes deployments
// to bring them into agreement with configuration.
package main

import (
	"flag"
	"log"
	"net/http"
	"strings"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	registry "github.com/googlecloudrobotics/core/src/go/pkg/apis/registry/v1alpha1"
	"github.com/googlecloudrobotics/core/src/go/pkg/controller/approllout"
	"github.com/googlecloudrobotics/core/src/go/pkg/controller/chartassignment"
	"github.com/pkg/errors"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/client-go/kubernetes/scheme"
	"k8s.io/client-go/rest"
	"k8s.io/helm/pkg/chartutil"
	"k8s.io/helm/pkg/strvals"
	"sigs.k8s.io/controller-runtime/pkg/manager"
	ctrllog "sigs.k8s.io/controller-runtime/pkg/runtime/log"
	"sigs.k8s.io/controller-runtime/pkg/runtime/signals"
	"sigs.k8s.io/controller-runtime/pkg/webhook"
)

var (
	cluster = flag.String("cluster", "cloud",
		"Name of the master's cluster")

	tillerHost = flag.String("tiller-host", chartassignment.DefaultTillerHost,
		"Host of Tiller")

	namespace = flag.String("namespace", "default",
		"Namespace of the processes' pod.")

	labels = flag.String("labels", "",
		"Labels of the processes' pod formatted as key1=value1,key2=value2")

	params = flag.String("params", "",
		"Helm configuration parameters formatted as name=value,topname.subname=value")

	webhookPort = flag.Int("webhook-port", 9876,
		"Listening port of the custom resource webhook")

	useSynk = flag.Bool("use-synk", false,
		"Install Helm charts with Synk")
)

func main() {
	log.SetFlags(log.Lshortfile | log.LstdFlags)
	flag.Parse()

	kubernetesConfig, err := rest.InClusterConfig()
	if err != nil {
		log.Fatalf("Failed to initialize Kubernetes config: %v", err)
	}

	helmParams, err := strvals.ParseString(*params)
	if err != nil {
		log.Fatalln("invalid Helm parameters:", err)
	}

	if err := setupAppV2(kubernetesConfig, helmParams); err != nil {
		log.Fatalln(err)
	}

	// Run a k8s liveness probe in the main thread.
	http.HandleFunc("/healthz", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "text/plain")
		w.Write([]byte("ok"))
	})
	log.Fatal(http.ListenAndServe(":8080", nil))
}

func setupAppV2(cfg *rest.Config, params map[string]interface{}) error {
	ctrllog.SetLogger(ctrllog.ZapLogger(true))

	sc := runtime.NewScheme()
	scheme.AddToScheme(sc)
	apps.AddToScheme(sc)
	registry.AddToScheme(sc)

	mgr, err := manager.New(cfg, manager.Options{Scheme: sc})
	if err != nil {
		return errors.Wrap(err, "create controller manager")
	}
	selectors, err := parseLabels(*labels)
	if err != nil {
		return err
	}
	srv, err := webhook.NewServer("chartassignment-admission-server", mgr, webhook.ServerOptions{
		Port: int32(*webhookPort),
		BootstrapOptions: &webhook.BootstrapOptions{
			Service: &webhook.Service{
				Namespace: *namespace,
				Name:      "cloudrobotics-apps-admission",
				Selectors: selectors,
			},
		},
	})
	if err != nil {
		return errors.Wrap(err, "create webhook server")
	}
	if err := chartassignment.Add(mgr, *cluster, *tillerHost, *useSynk); err != nil {
		return errors.Wrap(err, "add ChartAssignment controller")
	}
	if err := approllout.Add(mgr, chartutil.Values(params)); err != nil {
		return errors.Wrap(err, "add AppRollout controller")
	}

	chartassignmentValidation, err := chartassignment.NewValidationWebhook(mgr)
	if err != nil {
		return errors.Wrap(err, "create ChartAssignment validation webhook")
	}
	approlloutValidation, err := approllout.NewValidationWebhook(mgr)
	if err != nil {
		return errors.Wrap(err, "create AppRollout validation webhook")
	}
	if err := srv.Register(chartassignmentValidation, approlloutValidation); err != nil {
		return errors.Wrap(err, "register webhooks")
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
