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

package main

import (
	"testing"

	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/client-go/kubernetes/fake"
)

func TestWriteAlertmanagerTokenSucceeds(t *testing.T) {
	configYaml := `global:
  other: a
`
	clientset := fake.NewSimpleClientset()
	s := &corev1.Secret{
		ObjectMeta: metav1.ObjectMeta{
			Name:      "alertmanager-kube-prometheus",
			Namespace: "monitoring",
		},
		Data: map[string][]byte{
			"alertmanager.yaml": []byte(configYaml),
		},
	}
	_, err := clientset.CoreV1().Secrets("monitoring").Create(s)
	if err != nil {
		t.Errorf("Unexpected error: %v", err)
	}
	if err := writeAlertmanagerToken(clientset.CoreV1(), "mytoken"); err != nil {
		t.Errorf("Unexpected error: %v", err)
	}
	written, err := clientset.CoreV1().Secrets("monitoring").Get("alertmanager-kube-prometheus", metav1.GetOptions{})
	if err != nil {
		t.Errorf("Unexpected error: %v", err)
	}
	expectedYaml := `global:
  http_config:
    bearer_token: mytoken
  other: a
`
	if want, got := expectedYaml, string(written.Data["alertmanager.yaml"]); want != got {
		t.Errorf("Updated token yaml is wrong; want %s; got %s", want, got)
	}
}

func TestWriteAlertmanagerTokenRespectsTypeMismatch(t *testing.T) {
	configYaml := `global:
  http_config:
  - list
`
	clientset := fake.NewSimpleClientset()
	s := &corev1.Secret{
		ObjectMeta: metav1.ObjectMeta{
			Name:      "alertmanager-kube-prometheus",
			Namespace: "monitoring",
		},
		Data: map[string][]byte{
			"alertmanager.yaml": []byte(configYaml),
		},
	}
	_, err := clientset.CoreV1().Secrets("monitoring").Create(s)
	if err != nil {
		t.Errorf("Unexpected error: %v", err)
	}
	if err := writeAlertmanagerToken(clientset.CoreV1(), "mytoken"); err == nil {
		t.Errorf("Got no error")
	}
}
