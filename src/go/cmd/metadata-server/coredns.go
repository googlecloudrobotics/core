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
	"fmt"
	"regexp"
	"strings"

	"k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/client-go/kubernetes"
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
	hostsStart        = "    hosts {\n        "
	hostsStartPatched = `    hosts hosts metadata.google.internal host.minikube.internal {
        169.254.169.254 metadata.google.internal
        `
)

var (
	hostsHostMinikubeInternalPattern = regexp.MustCompile(hostsStart + `([.\d]+ host\.minikube\.internal\n\s{8}fallthrough\n\s{4}\})`)
	hostsAnyPattern                  = regexp.MustCompile(`hosts [^{]*{\n`)
)

func getCorefile(ctx context.Context, k8s kubernetes.Interface) (*v1.ConfigMap, error) {
	cm, err := k8s.CoreV1().ConfigMaps(configMapNamespace).Get(ctx, configMapName, metav1.GetOptions{})
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

func writeCorefile(ctx context.Context, k8s kubernetes.Interface, cm *v1.ConfigMap) error {
	_, err := k8s.CoreV1().ConfigMaps(configMapNamespace).Update(ctx, cm, metav1.UpdateOptions{})
	return err
}

// PatchCorefile reads the CoreDNS config map and patches the Corefile to resolve
// metadata.google.internal to 169.254.169.254.
func PatchCorefile(ctx context.Context, k8s kubernetes.Interface) error {
	cm, err := getCorefile(ctx, k8s)
	if err != nil {
		return err
	}
	// must be idempotent
	if strings.Contains(cm.Data[corefileName], zoneStartPatched) || strings.Contains(cm.Data[corefileName], hostsStartPatched) {
		return nil
	}
	if m := hostsHostMinikubeInternalPattern.FindStringIndex(cm.Data[corefileName]); m != nil {
		cm.Data[corefileName] = hostsHostMinikubeInternalPattern.ReplaceAllString(cm.Data[corefileName], hostsStartPatched+"$1")
	} else {
		cm.Data[corefileName] = strings.Replace(cm.Data[corefileName], zoneStart, zoneStartPatched, 1)
	}
	if len(hostsAnyPattern.FindAllString(cm.Data[corefileName], -1)) > 1 {
		return fmt.Errorf("multiple hosts entries after patching, please check the input")
	}

	return writeCorefile(ctx, k8s, cm)
}

// RevertCorefile undoes the effect of PatchCorefile.
func RevertCorefile(ctx context.Context, k8s kubernetes.Interface) error {
	cm, err := getCorefile(ctx, k8s)
	if err != nil {
		return err
	}
	containsZoneStartPatch := strings.Contains(cm.Data[corefileName], zoneStartPatched)
	containsHostsStartPatch := strings.Contains(cm.Data[corefileName], hostsStartPatched)
	// must be idempotent
	if !containsZoneStartPatch && !containsHostsStartPatch {
		return nil
	}
	if containsZoneStartPatch && !containsHostsStartPatch {
		cm.Data[corefileName] = strings.Replace(cm.Data[corefileName], zoneStartPatched, zoneStart, 1)
	} else if !containsZoneStartPatch && containsHostsStartPatch {
		cm.Data[corefileName] = strings.Replace(cm.Data[corefileName], hostsStartPatched, hostsStart, 1)
	} else {
		return fmt.Errorf("cannot contain both patches")
	}
	return writeCorefile(ctx, k8s, cm)
}
