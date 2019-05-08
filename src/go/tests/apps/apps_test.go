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
	"fmt"
	"os"
	"testing"
	"time"

	"github.com/cenkalti/backoff"
	roboapps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"github.com/googlecloudrobotics/core/src/go/pkg/kubetest"
	apps "k8s.io/api/apps/v1"
	apierrors "k8s.io/apimachinery/pkg/api/errors"
	"sigs.k8s.io/controller-runtime/pkg/client"
)

func TestAll(t *testing.T) {
	env := kubetest.New(t, kubetest.Config{
		Clusters: []kubetest.ClusterConfig{
			{Name: "robot", InstallHelm: true},
		},
		SchemeFunc: roboapps.AddToScheme,
	})
	defer env.Teardown()

	env.InstallChartArchive(
		"robot", "base-test", "default",
		"src/app_charts/base/base-test-0.0.1.tgz",
		map[string]string{
			"robot.name": "testbot",
			"registry":   os.Getenv("REGISTRY"),
		},
	)
	if err := backoff.Retry(
		func() error {
			return kubetest.DeploymentReady(env.Ctx(), env.Client("robot"), "default", "robot-master")
		},
		backoff.WithMaxRetries(backoff.NewConstantBackOff(3*time.Second), 40),
	); err != nil {
		t.Errorf("wait for robot-master: %s", err)
		t.Fatalf("ACCESS_TOKEN set?")
	}

	env.Run(
		testCreateChartAssignment_WithChartReference_Works,
	)
}

func testCreateChartAssignment_WithChartReference_Works(t *testing.T, f *kubetest.Fixture) {
	robot := f.Client("robot")

	tmpl := `
apiVersion: apps.cloudrobotics.com/v1alpha1
kind: ChartAssignment
metadata:
  name: {{ .name }}
spec:
  clusterName: testbot
  namespaceName: {{ .namespace }}
  chart:
    repository: https://kubernetes-charts.storage.googleapis.com/
    name: oauth2-proxy
    version: 0.10.0
    values:
      fullnameOverride: test
`
	data := map[string]string{
		"name":      f.Uniq("example"),
		"namespace": f.Uniq("ns"),
	}
	var ca roboapps.ChartAssignment
	f.FromYAML(tmpl, data, &ca)

	if err := robot.Create(f.Ctx(), &ca); err != nil {
		t.Fatalf("create ChartAssignment: %s", err)
	}

	// We should find a deployment in its own namespace now.
	if err := backoff.Retry(
		func() error {
			// Wait for deployment to be created.
			var dep apps.Deployment
			err := robot.Get(f.Ctx(), client.ObjectKey{
				Namespace: data["namespace"],
				Name:      "test",
			}, &dep)
			if apierrors.IsNotFound(err) {
				t.Logf("Deployment does not exist yet")
				return fmt.Errorf("deployment does not exist yet")
			}
			if err != nil {
				return backoff.Permanent(err)
			}
			// Ensure ChartAssignment was deployed successfully.
			if err = robot.Get(f.Ctx(), f.ObjectKey(&ca), &ca); err != nil {
				return backoff.Permanent(err)
			}
			if ca.Status.Phase == roboapps.ChartAssignmentPhaseSettled {
				return nil
			}
			// Chart should've been deployed exactly once.
			if ca.Status.Helm.Revision == 1 {
				return nil
			}
			t.Logf("Phase: %s, Revision: %d", ca.Status.Phase, ca.Status.Helm.Revision)
			return fmt.Errorf("revision != 1")
		},
		backoff.WithMaxRetries(backoff.NewConstantBackOff(time.Second), 60),
	); err != nil {
		t.Fatalf("wait for mysql deployment: %s", err)
	}
}
