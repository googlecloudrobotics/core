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
	"testing"

	"github.com/googlecloudrobotics/core/src/go/pkg/kubeutils"
)

func TestCloudClusterAppStatus(t *testing.T) {
	kubernetesContext, err := kubeutils.GetCloudKubernetesContext()
	if err != nil {
		t.Fatal(err)
	}
	k8sCfg, err := kubeutils.LoadOutOfClusterConfig(kubernetesContext)
	if err != nil {
		t.Fatalf("Loading of kubernetes config failed: %v", err)
	}

	if err := checkAppRollouts(t.Context(), kubernetesContext, k8sCfg); err != nil {
		t.Error(err)
	}
}

func TestKubernetesCloudClusterStatus(t *testing.T) {
	ctx := t.Context()
	kubernetesCloudContext, err := kubeutils.GetCloudKubernetesContext()
	if err != nil {
		t.Fatal(err)
	}

	if err := checkHealthOfKubernetesCluster(ctx, kubernetesCloudContext); err != nil {
		t.Errorf("Cloud cluster %s: %v", kubernetesCloudContext, err)
	}
}

func TestKubernetesRobotClusterStatus(t *testing.T) {
	ctx := t.Context()
	kubernetesRobotContext, err := kubeutils.GetRobotKubernetesContext()
	if err != nil {
		t.Fatal(err)
	}

	if err := checkHealthOfKubernetesCluster(ctx, kubernetesRobotContext); err != nil {
		t.Errorf("Robot cluster %s: %v", kubernetesRobotContext, err)
	}
}
