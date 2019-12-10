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

package apps

import (
	"os"
	"testing"
	"time"

	"github.com/cenkalti/backoff"
	crcapps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"github.com/googlecloudrobotics/core/src/go/pkg/kubetest"
	apps "k8s.io/api/apps/v1"
	"sigs.k8s.io/controller-runtime/pkg/client"
)

const (
	robotClusterName = "robot"
)

func TestAll(t *testing.T) {
	env := kubetest.New(t, kubetest.Config{
		Clusters: []kubetest.ClusterConfig{
			{Name: robotClusterName},
		},
		SchemeFunc: crcapps.AddToScheme,
	})
	defer env.Teardown()

	env.InstallChartArchive(
		robotClusterName, "base-test", "default",
		"src/app_charts/base/base-test-0.0.1.tgz",
		map[string]string{
			"robot.name":      robotClusterName,
			"registry":        os.Getenv("REGISTRY"),
			"webhook.enabled": "false",
		},
	)
	if err := backoff.Retry(
		func() error {
			return kubetest.DeploymentReady(env.Ctx(), env.Client(robotClusterName), "default", "robot-master")
		},
		backoff.WithMaxRetries(backoff.NewConstantBackOff(3*time.Second), 40),
	); err != nil {
		t.Errorf("wait for robot-master: %s", err)
		t.Fatalf("maybe REGISTRY or ACCESS_TOKEN is not set?")
	}

	env.Run(
		testCreateChartAssignment_WithChartReference_Works,
	)
}

func testCreateChartAssignment_WithChartReference_Works(t *testing.T, f *kubetest.Fixture) {
	robot := f.Client(robotClusterName)

	tmpl := `
apiVersion: apps.cloudrobotics.com/v1alpha1
kind: ChartAssignment
metadata:
  name: {{ .name }}
  namespace: default
spec:
  clusterName: {{ .cluster }}
  namespaceName: {{ .namespace }}
  chart:
    repository: https://kubernetes-charts.storage.googleapis.com/
    name: oauth2-proxy
    version: 0.10.0
    values:
      fullnameOverride: test
`
	data := map[string]string{
		"cluster":   robotClusterName,
		"name":      f.Uniq("example"),
		"namespace": f.Uniq("ns"),
	}
	var ca crcapps.ChartAssignment
	f.FromYAML(tmpl, data, &ca)

	if err := robot.Create(f.Ctx(), &ca); err != nil {
		t.Fatalf("create ChartAssignment: %s", err)
	}

	if err := backoff.Retry(
		f.ChartAssignmentHasStatus(&ca, crcapps.ChartAssignmentPhaseSettled),
		backoff.WithMaxRetries(backoff.NewConstantBackOff(time.Second), 60),
	); err != nil {
		t.Fatalf("wait for chart assignment settled: %s", err)
	}

	// We should find a deployment in its own namespace now.
	var dep apps.Deployment
	if err := robot.Get(f.Ctx(), client.ObjectKey{
		Namespace: data["namespace"],
		Name:      "test",
	}, &dep); err != nil {
		t.Errorf("failed to get deployment: %s", err)
	}
	// Chart should've been deployed exactly once.
	if want, got := int64(1), ca.Status.ObservedGeneration; want != got {
		t.Errorf("want ca.Status.ObservedGeneration == %d, got %d", want, got)
	}
}
