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

// The grafana operator collates all config maps with dashboards into one single map.
//
// Kube-prometheus' Grafana expects a single config map with all dashboards, but
// we'd like each constellation to bring its own dashboards. This process watches
// all config maps with label "grafana": "kube-prometheus" and collates them into
// one map "extra-grafana-dashboards".
package main

import (
	"context"
	"encoding/json"
	"flag"
	"fmt"
	"log"
	"net/http"
	"time"

	corev1 "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/api/errors"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/labels"
	"k8s.io/client-go/informers"
	"k8s.io/client-go/kubernetes"
	typedv1 "k8s.io/client-go/kubernetes/typed/core/v1"
	"k8s.io/client-go/rest"
	"k8s.io/client-go/tools/cache"
)

var (
	grafanaLabel    = flag.String("grafana-label", "kube-prometheus", "Act on config maps with this grafana label")
	targetNamespace = flag.String("target-namespace", "monitoring", "Namespace of the generated config map")
	targetName      = flag.String("target-name", "extra-grafana-dashboards", "Name of the generated config map")
)

type MapWriter struct {
	files  map[string]map[string]string
	client typedv1.ConfigMapInterface
}

// MaybeConvertToApiDashboard converts any Grafana GUI JSON to API JSON.
// If you export a dashboard from Grafana's GUI, you'll get a slightly
// different format than the import API expects. The GUI JSON needs to be
// wrapped in a "dashboard" element and decorated a bit. Do so in the
// operator so that folks don't have to think about it.
func MaybeConvertToApiDashboard(input string) (string, error) {
	var parsed map[string]interface{}
	err := json.Unmarshal([]byte(input), &parsed)
	if err != nil {
		return "", fmt.Errorf("JSON parse error: %v", err)
	}
	if parsed["dashboard"] != nil {
		// Already in API format
		return input, nil
	}
	if parsed["title"] == nil {
		// That's also not the GUI export format
		return "", fmt.Errorf("Unrecognized dashboard JSON, needs either dashboard or title")
	}
	wrapped := make(map[string]interface{})
	wrapped["dashboard"] = parsed
	wrapped["inputs"] = []map[string]string{{
		"name":     "DS_PROMETHEUS",
		"pluginId": "prometheus",
		"type":     "datasource",
		"value":    "prometheus",
	}}
	wrapped["overwrite"] = true
	result, err := json.Marshal(wrapped)
	if err != nil {
		return "", fmt.Errorf("JSON serialization error: %v", err)
	}
	return string(result), nil
}

func NewMapWriter(corev1client typedv1.CoreV1Interface) *MapWriter {
	return &MapWriter{
		files:  make(map[string]map[string]string),
		client: corev1client.ConfigMaps(*targetNamespace),
	}
}

func (mw *MapWriter) tryToWriteConfigMap() error {
	writeFunc := mw.client.Update
	cm, err := mw.client.Get(*targetName, metav1.GetOptions{})
	if err != nil {
		status, isStatus := err.(*errors.StatusError)
		if isStatus && status.ErrStatus.Code == http.StatusNotFound {
			cm = &corev1.ConfigMap{
				ObjectMeta: metav1.ObjectMeta{
					Name:      *targetName,
					Namespace: *targetNamespace,
				},
			}
			writeFunc = mw.client.Create
		} else {
			return err
		}
	}
	cm.Data = make(map[string]string)
	for inputName, input := range mw.files {
		for fileName, file := range input {
			converted, err := MaybeConvertToApiDashboard(file)
			if err != nil {
				log.Printf("Ignoring invalid dashboard %s in %s: %v", fileName, inputName, err)
			}
			cm.Data[fmt.Sprintf("%s-%s", inputName, fileName)] = converted
		}
	}
	_, err = writeFunc(cm)
	return err
}

func (mw *MapWriter) WriteConfigMap() {
	for err := mw.tryToWriteConfigMap(); err != nil; err = mw.tryToWriteConfigMap() {
		log.Printf("Failed to write result config map: %v", err)
	}
	log.Printf("Updated target config map")
}

func (mw *MapWriter) Update(oldMap *corev1.ConfigMap, newMap *corev1.ConfigMap) {
	updated := false
	if oldMap != nil && oldMap.ObjectMeta.Labels["grafana"] == "kube-prometheus" {
		mw.files[fmt.Sprintf("%s-%s", oldMap.ObjectMeta.Namespace, oldMap.ObjectMeta.Name)] = nil
		updated = true
	}
	if newMap != nil && newMap.ObjectMeta.Labels["grafana"] == "kube-prometheus" {
		mw.files[fmt.Sprintf("%s-%s", newMap.ObjectMeta.Namespace, newMap.ObjectMeta.Name)] = newMap.Data
		updated = true
	}
	if updated {
		mw.WriteConfigMap()
	}
}

func main() {
	flag.Parse()
	ctx := context.Background()

	config, err := rest.InClusterConfig()
	if err != nil {
		log.Fatalf("Failed to build REST config for K8s: %v", err)
	}
	clientset, err := kubernetes.NewForConfig(config)
	if err != nil {
		log.Fatalf("Failed to build clientset: %v", err)
	}
	mw := NewMapWriter(clientset.CoreV1())
	mw.WriteConfigMap()
	eventHandler := cache.ResourceEventHandlerFuncs{
		AddFunc: func(obj interface{}) {
			log.Printf("Handling creation of %s", obj.(*corev1.ConfigMap).ObjectMeta.Name)
			mw.Update(nil, obj.(*corev1.ConfigMap))
		},
		UpdateFunc: func(oldObj, newObj interface{}) {
			log.Printf("Handling update of %s", newObj.(*corev1.ConfigMap).ObjectMeta.Name)
			mw.Update(oldObj.(*corev1.ConfigMap), newObj.(*corev1.ConfigMap))
		},
		DeleteFunc: func(obj interface{}) {
			log.Printf("Handling deletion of %s", obj.(*corev1.ConfigMap).ObjectMeta.Name)
			mw.Update(obj.(*corev1.ConfigMap), nil)
		},
	}

	selectorTweak := func(opts *metav1.ListOptions) {
		label := labels.SelectorFromSet(labels.Set(map[string]string{"grafana": "kube-prometheus"}))
		opts.LabelSelector = label.String()
	}
	factory := informers.NewSharedInformerFactoryWithOptions(
		clientset,
		10*time.Minute,
		informers.WithTweakListOptions(selectorTweak))
	informer := factory.Core().V1().ConfigMaps().Informer()

	// Careful: Initializing informers is intricate. See
	// https://engineering.bitnami.com/articles/a-deep-dive-into-kubernetes-controllers.html
	go func() {
		log.Printf("Syncing cache for input config maps")
		ok := cache.WaitForCacheSync(ctx.Done(), informer.HasSynced)
		if !ok {
			log.Fatalf("WaitForCacheSync failed")
		}

		informer.AddEventHandler(&eventHandler)
	}()

	go keepAlertmanagerTokenUpdated(clientset.CoreV1())

	log.Printf("Running informer")
	informer.Run(ctx.Done())
}
