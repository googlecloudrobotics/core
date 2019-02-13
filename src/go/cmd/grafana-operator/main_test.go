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
	"strings"
	"testing"

	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/client-go/kubernetes/fake"
)

func TestCreation(t *testing.T) {
	clientset := fake.NewSimpleClientset()
	cm := &corev1.ConfigMap{
		ObjectMeta: metav1.ObjectMeta{
			Name:      "foobar",
			Namespace: "default",
			Labels:    map[string]string{"grafana": "kube-prometheus"},
		},
		Data: map[string]string{
			"cartographer-dashboard.json": "{ \"dashboard\": {} }",
		},
	}
	mw := NewMapWriter(clientset.CoreV1())
	mw.Update(nil, cm)
	written, err := clientset.CoreV1().ConfigMaps("monitoring").Get("extra-grafana-dashboards", metav1.GetOptions{})
	if err != nil {
		t.Errorf("No config map generated: %v", err)
	}
	if want, got := 1, len(written.Data); want != got {
		t.Errorf("Wrong map length; want %d; got %d", want, got)
	}
	if want, got := "{ \"dashboard\": {} }", written.Data["default-foobar-cartographer-dashboard.json"]; want != got {
		t.Errorf("Wrong map entry; want %s; got %s", want, got)
	}
}

func TestUpdate(t *testing.T) {
	clientset := fake.NewSimpleClientset()
	cmOld := &corev1.ConfigMap{
		ObjectMeta: metav1.ObjectMeta{
			Name:      "foobar",
			Namespace: "default",
			Labels:    map[string]string{"grafana": "kube-prometheus"},
		},
		Data: map[string]string{
			"cartographer-dashboard.json": "{ \"dashboard\": {} }",
		},
	}
	cmNew := cmOld.DeepCopy()
	cmNew.Data["cartographer-dashboard.json"] = "{ \"title\": \"foo\" }"
	mw := NewMapWriter(clientset.CoreV1())
	mw.Update(nil, cmOld)
	mw.Update(cmOld, cmNew)
	written, err := clientset.CoreV1().ConfigMaps("monitoring").Get("extra-grafana-dashboards", metav1.GetOptions{})
	if err != nil {
		t.Errorf("No config map generated: %v", err)
	}
	if want, got := 1, len(written.Data); want != got {
		t.Errorf("Wrong map length; want %d; got %d", want, got)
	}
	if want, got := "pluginId", written.Data["default-foobar-cartographer-dashboard.json"]; strings.Contains(want, got) {
		t.Errorf("Wrong map entry; want substring %s; got %s", want, got)
	}
}
