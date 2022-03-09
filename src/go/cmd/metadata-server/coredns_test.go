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
	"context"
	"strings"
	"testing"

	"k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"

	"k8s.io/client-go/kubernetes"
	"k8s.io/client-go/kubernetes/fake"
)

const (
	defaultCorefileBeforeMinikube121 = `.:53 {
    whoami
}
`
	modifiedCorefileBeforeMinikube121 = `.:53 {
    hosts hosts metadata.google.internal {
        169.254.169.254 metadata.google.internal
        fallthrough
    }
    whoami
}
`
	defaultCorefileAfterMinikube121 = `.:53 {
    whoami
    hosts {
       127.0.0.1 host.minikube.internal
       fallthrough
    }
}
`
	modifiedCorefileAfterMinikube121 = `.:53 {
    whoami
    hosts hosts metadata.google.internal host.minikube.internal {
        169.254.169.254 metadata.google.internal
       127.0.0.1 host.minikube.internal
       fallthrough
    }
}
`
	defaultCorefileUnexpected = `.:53 {
    whoami
    hosts {
        127.0.0.1 host2.minikube.internal
        fallthrough
    }
}
`
	differentIp = "1.2.3.456"
)

func createCorefile(t *testing.T, k8s kubernetes.Interface, corefileData string) {
	if _, err := k8s.CoreV1().ConfigMaps(configMapNamespace).Create(
		context.Background(),
		&v1.ConfigMap{
			Data: map[string]string{
				corefileName: corefileData,
			},
			ObjectMeta: metav1.ObjectMeta{
				Name: configMapName,
			},
		},
		metav1.CreateOptions{}); err != nil {
		t.Errorf("error creating ConfigMap %s: %v", configMapName, err)
	}
}

func readCorefile(t *testing.T, k8s kubernetes.Interface) string {
	cm, err := k8s.CoreV1().ConfigMaps(configMapNamespace).Get(context.Background(), configMapName, metav1.GetOptions{})
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
	tests := []struct {
		desc  string
		input string
		want  string
	}{
		{
			"default corefile without host.minikube.internal entry",
			defaultCorefileBeforeMinikube121,
			modifiedCorefileBeforeMinikube121,
		},
		{
			"default corefile with host.minikube.internal entry",
			defaultCorefileAfterMinikube121,
			modifiedCorefileAfterMinikube121,
		},
		{
			"default corefile with host.minikube.internal entry, different IP",
			strings.Replace(defaultCorefileAfterMinikube121, "127.0.0.1", differentIp, 1),
			strings.Replace(modifiedCorefileAfterMinikube121, "127.0.0.1", differentIp, 1),
		},
	}

	for _, tc := range tests {
		t.Run(tc.desc, func(t *testing.T) {
			ctx := context.Background()
			k8s := fake.NewSimpleClientset()
			createCorefile(t, k8s, tc.input)

			if err := PatchCorefile(ctx, k8s); err != nil {
				t.Errorf("error in PatchCorefile: %v", err)
			}
			if got := readCorefile(t, k8s); got != tc.want {
				t.Errorf(`want readCorefile(t, k8s) = %q, got %q`, tc.want, got)
			}

			// Check that a second patch has no effect.
			if err := PatchCorefile(ctx, k8s); err != nil {
				t.Errorf("error in second PatchCorefile: %v", err)
			}
			if got := readCorefile(t, k8s); got != tc.want {
				t.Errorf(`after second patch, want readCorefile(t, k8s) = %q, got %q`, tc.want, got)
			}

			// Check that reverting undoes the change.
			if err := RevertCorefile(ctx, k8s); err != nil {
				t.Errorf("error in RevertCorefile: %v", err)
			}
			if got := readCorefile(t, k8s); got != tc.input {
				t.Errorf(`after revert, want readCorefile(t, k8s) = %q, got %q`, tc.input, got)
			}

			// Check that a second revert has no effect.
			if err := RevertCorefile(ctx, k8s); err != nil {
				t.Errorf("error in second RevertCorefile: %v", err)
			}
			if got := readCorefile(t, k8s); got != tc.input {
				t.Errorf(`after second revert, want readCorefile(t, k8s) = %q, got %q`, tc.input, got)
			}
		})
	}
}

func TestPatchCorefileUnexpected(t *testing.T) {
	ctx := context.Background()
	k8s := fake.NewSimpleClientset()
	createCorefile(t, k8s, defaultCorefileUnexpected)

	if err := PatchCorefile(ctx, k8s); err == nil {
		t.Error("PatchCorefile() succeeded with invalid input, wanted error")
	}
	if got := readCorefile(t, k8s); got != defaultCorefileUnexpected {
		t.Errorf(`unexpected input should not be modified, want readCorefile(t, k8s) = %q, got %q`, defaultCorefileUnexpected, got)
	}
}
