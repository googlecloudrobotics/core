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
	"k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"testing"

	"k8s.io/client-go/kubernetes"
	"k8s.io/client-go/kubernetes/fake"
)

const (
	defaultCorefile = `.:53 {
    whoami
}
`
	modifiedCorefile = `.:53 {
    hosts hosts metadata.google.internal {
        169.254.169.254 metadata.google.internal
        fallthrough
    }
    whoami
}
`
)

func createCorefile(t *testing.T, k8s kubernetes.Interface) {
	if _, err := k8s.CoreV1().ConfigMaps(configMapNamespace).Create(&v1.ConfigMap{
		Data: map[string]string{
			corefileName: defaultCorefile,
		},
		ObjectMeta: metav1.ObjectMeta{
			Name: configMapName,
		},
	}); err != nil {
		t.Errorf("error creating ConfigMap %s: %v", configMapName, err)
	}
}

func readCorefile(t *testing.T, k8s kubernetes.Interface) string {
	cm, err := k8s.CoreV1().ConfigMaps(configMapNamespace).Get(configMapName, metav1.GetOptions{})
	if err != nil {
		t.Errorf("error reading ConfigMap coredns: %v", err)
		return ""
	}
	data, ok := cm.Data[corefileName]
	if !ok {
		t.Errorf("ConfigMap %s doesn't contain key %s", configMapName, corefileName)
		return ""
	}
	return data
}

func TestPatchCorefile(t *testing.T) {
	k8s := fake.NewSimpleClientset()
	createCorefile(t, k8s)

	if err := PatchCorefile(k8s); err != nil {
		t.Errorf("error in PatchCorefile: %v", err)
	}
	if got := readCorefile(t, k8s); got != modifiedCorefile {
		t.Errorf(`want readCorefile(t, k8s) = %q, got %q`, modifiedCorefile, got)
	}

	// Check that a second patch has no effect.
	if err := PatchCorefile(k8s); err != nil {
		t.Errorf("error in second PatchCorefile: %v", err)
	}
	if got := readCorefile(t, k8s); got != modifiedCorefile {
		t.Errorf(`after second patch, want readCorefile(t, k8s) = %q, got %q`, modifiedCorefile, got)
	}

	// Check that reverting undoes the change.
	if err := RevertCorefile(k8s); err != nil {
		t.Errorf("error in RevertCorefile: %v", err)
	}
	if got := readCorefile(t, k8s); got != defaultCorefile {
		t.Errorf(`after revert, want readCorefile(t, k8s) = %q, got %q`, defaultCorefile, got)
	}

	// Check that a second revert has no effect.
	if err := RevertCorefile(k8s); err != nil {
		t.Errorf("error in second RevertCorefile: %v", err)
	}
	if got := readCorefile(t, k8s); got != defaultCorefile {
		t.Errorf(`after second revert, want readCorefile(t, k8s) = %q, got %q`, defaultCorefile, got)
	}
}
