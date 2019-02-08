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
	"strings"
	"time"

	pb "src/proto/registry"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"github.com/googlecloudrobotics/core/src/go/pkg/controller/chartassignment"
	"github.com/googlecloudrobotics/core/src/go/pkg/helm"
	"github.com/pkg/errors"
	"golang.org/x/net/context"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/client-go/dynamic"
	"k8s.io/client-go/kubernetes/scheme"
	"k8s.io/client-go/rest"
	"k8s.io/helm/pkg/chartutil"
	"k8s.io/helm/pkg/strvals"
	"sigs.k8s.io/controller-runtime/pkg/manager"
	ctrllog "sigs.k8s.io/controller-runtime/pkg/runtime/log"
	"sigs.k8s.io/controller-runtime/pkg/runtime/signals"
	"sigs.k8s.io/controller-runtime/pkg/webhook"
)

const (
	errBackoffInterval = 10 * time.Second
	pollInterval       = 30 * time.Second
)

var (
	enableAppV2 = flag.Bool("enable-app-v2", false,
		"Enable the application layer v2 controllers")

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
)

func syncAppStateOnce(ctx context.Context, robotClient dynamic.ResourceInterface, h *helm.Helm, robotName string) error {
	robot, err := robotClient.Get(robotName, metav1.GetOptions{})
	if err != nil {
		return fmt.Errorf("failed to get Robot '%s': %v", robotName, err)
	}
	if err := h.InstallApps(ctx, "cloud-robotics-apps", "default", pb.InstallationTarget_ROBOT, []unstructured.Unstructured{*robot}); err != nil {
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
		log.Fatalf("expect ROBOT_NAME environment var to be set to an non-empty string")
	}

	config, err := rest.InClusterConfig()
	if err != nil {
		log.Fatalln(err)
	}
	k8sClient, err := dynamic.NewForConfig(config)
	if err != nil {
		log.Fatalln(err)
	}

	helmParams, err := strvals.Parse(*params)
	if err != nil {
		log.Fatalln("invalid Helm parameters:", err)
	}

	if *enableAppV2 {
		if err := setupAppV2(config, robotName, helmParams); err != nil {
			log.Fatalln(err)
		}
	}

	h, err := helm.NewHelm(k8sClient, helmParams)
	if err != nil {
		log.Fatalln(err.Error())
	}

	go syncAppStateTask(ctx, k8sClient, h, robotName)

	// Run a k8s liveness probe in the main thread.
	http.HandleFunc("/healthz", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "text/plain")
		w.Write([]byte("ok"))
	})
	log.Fatal(http.ListenAndServe(":8080", nil))
}

func setupAppV2(cfg *rest.Config, cluster string, params map[string]interface{}) error {
	ctrllog.SetLogger(ctrllog.ZapLogger(true))

	sc := runtime.NewScheme()
	scheme.AddToScheme(sc)
	apps.AddToScheme(sc)

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
	if err := chartassignment.Add(mgr, srv, cluster, *tillerHost, chartutil.Values(params)); err != nil {
		return errors.Wrap(err, "add ChartAssignment controller")
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
