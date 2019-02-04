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
	"fmt"
	"k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/client-go/kubernetes"
	"strings"
)

const (
	configMapName      = "coredns"
	configMapNamespace = "kube-system"
	corefileName       = "Corefile"
	zoneStart          = ".:53 {\n"
	zoneStartPatched   = `.:53 {
    hosts hosts metadata.google.internal {
        169.254.169.254 metadata.google.internal
        fallthrough
    }
`
)

func getCorefile(k8s kubernetes.Interface) (*v1.ConfigMap, error) {
	cm, err := k8s.CoreV1().ConfigMaps(configMapNamespace).Get(configMapName, metav1.GetOptions{})
	if err != nil {
		return nil, fmt.Errorf("failed to get ConfigMap %s: %v", configMapName, err)
	}
	if _, ok := cm.Data[corefileName]; !ok {
		return nil, fmt.Errorf("ConfigMap %s doesn't contain key %s", configMapName, corefileName)
	}
	if !strings.Contains(cm.Data[corefileName], zoneStart) {
		return nil, fmt.Errorf("zone start %q not found in Corefile", zoneStart)
	}
	return cm, nil
}

func writeCorefile(k8s kubernetes.Interface, cm *v1.ConfigMap) error {
	_, err := k8s.CoreV1().ConfigMaps(configMapNamespace).Update(cm)
	return err
}

// PatchCorefile reads the CoreDNS config map and patches the Corefile to resolve
// metadata.google.internal to 169.254.169.254.
func PatchCorefile(k8s kubernetes.Interface) error {
	cm, err := getCorefile(k8s)
	if err != nil {
		return err
	}
	if strings.Contains(cm.Data[corefileName], zoneStartPatched) {
		return nil
	}
	cm.Data[corefileName] = strings.Replace(cm.Data[corefileName], zoneStart, zoneStartPatched, 1)
	return writeCorefile(k8s, cm)
}

// RevertCorefile undoes the effect of PatchCorefile.
func RevertCorefile(k8s kubernetes.Interface) error {
	cm, err := getCorefile(k8s)
	if err != nil {
		return err
	}
	if !strings.Contains(cm.Data[corefileName], zoneStartPatched) {
		return nil
	}
	cm.Data[corefileName] = strings.Replace(cm.Data[corefileName], zoneStartPatched, zoneStart, 1)
	return writeCorefile(k8s, cm)
}
