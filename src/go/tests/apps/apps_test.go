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
	apierrors "k8s.io/apimachinery/pkg/api/errors"
	"sigs.k8s.io/controller-runtime/pkg/client"
)

const (
	robotClusterName = "robot"

	inlineChartTemplate = `
apiVersion: apps.cloudrobotics.com/v1alpha1
kind: ChartAssignment
metadata:
  name: {{ .name }}
  namespace: default
spec:
  clusterName: {{ .cluster }}
  namespaceName: {{ .namespace }}
  chart:
    inline: "{{ .chart }}"`
	goodDeployment = `
apiVersion: apps/v1
kind: Deployment
metadata:
  name: test
spec:
  replicas: 1
  selector:
    matchLabels:
      app: test
  template:
    metadata:
      labels:
        app: test
    spec:
      containers:
      - name: test
        image: "gcr.io/google-containers/busybox:latest"
        args: ["sleep", "999999999"]`
	deploymentWithBadLabels = `
apiVersion: apps/v1
kind: Deployment
metadata:
  name: test
spec:
  replicas: 1
  selector:
    matchLabels:
      app: test
  template:
    metadata:
      labels:
        app: this-label-does-not-match
    spec:
      containers:
      - name: test
        image: "gcr.io/google-containers/busybox:latest"
        args: ["sleep", "999999999"]`
	badJob = `
apiVersion: batch/v1
kind: Job
metadata:
  name: test
spec:
  template:
    metadata:
      labels:
        app: test
    spec:
      restartPolicy: Never
      containers:
      - name: test
        image: "gcr.io/google-containers/busybox:latest"
        args: ["false"]`
	goodJob = `
apiVersion: batch/v1
kind: Job
metadata:
  name: test
spec:
  template:
    metadata:
      labels:
        app: test
    spec:
      restartPolicy: Never
      containers:
      - name: test
        image: "gcr.io/google-containers/busybox:latest"
        args: ["true"]`
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
			return kubetest.DeploymentReady(env.Ctx(), env.Client(robotClusterName), "default", "chart-assignment-controller")
		},
		backoff.WithMaxRetries(backoff.NewConstantBackOff(3*time.Second), 40),
	); err != nil {
		t.Errorf("wait for chart-assignment-controller: %s", err)
		t.Fatalf("maybe REGISTRY or ACCESS_TOKEN is not set?")
	}

	env.Run(
		testCreateChartAssignment_WithChartReference_Works,
		testCreateChartAssignment_WithInlineChart_BecomesReady,
		testCreateChartAssignment_WithBadDeployment_BecomesFailed,
		testUpdateChartAssignment_WithFixedDeployment_BecomesReady,
		testUpdateChartAssignment_WithFixedJob_BecomesReady,
	)
}

func testCreateChartAssignment_WithChartReference_Works(t *testing.T, f *kubetest.Fixture) {
	robot := f.Client(robotClusterName)

	// for the version, run:
	// helm repo add oauth2-proxy https://oauth2-proxy.github.io/manifests
	// helm repo update
	// helm search oauth2-proxy/oauth2-proxy -l
	// Important: we need a V1 chart!
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
    repository: https://oauth2-proxy.github.io/manifests
    name: oauth2-proxy
    version: 3.2.6
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

func testCreateChartAssignment_WithInlineChart_BecomesReady(t *testing.T, f *kubetest.Fixture) {
	robot := f.Client(robotClusterName)

	data := map[string]string{
		"cluster":   robotClusterName,
		"name":      f.Uniq("example"),
		"namespace": f.Uniq("ns"),
		"chart":     kubetest.BuildInlineChart(t, "example", goodDeployment /*values=*/, ""),
	}
	var ca crcapps.ChartAssignment
	f.FromYAML(inlineChartTemplate, data, &ca)

	if err := robot.Create(f.Ctx(), &ca); err != nil {
		t.Fatalf("create ChartAssignment: %s", err)
	}

	if err := backoff.Retry(
		f.ChartAssignmentHasStatus(&ca, crcapps.ChartAssignmentPhaseReady),
		backoff.WithMaxRetries(backoff.NewConstantBackOff(time.Second), 60),
	); err != nil {
		t.Fatalf("wait for chart assignment ready: %s", err)
	}
}

func testCreateChartAssignment_WithBadDeployment_BecomesFailed(t *testing.T, f *kubetest.Fixture) {
	robot := f.Client(robotClusterName)

	data := map[string]string{
		"cluster":   robotClusterName,
		"name":      f.Uniq("example"),
		"namespace": f.Uniq("ns"),
		"chart":     kubetest.BuildInlineChart(t, "example", deploymentWithBadLabels /*values=*/, ""),
	}
	var ca crcapps.ChartAssignment
	f.FromYAML(inlineChartTemplate, data, &ca)

	if err := robot.Create(f.Ctx(), &ca); err != nil {
		t.Fatalf("create ChartAssignment: %s", err)
	}

	if err := backoff.Retry(
		f.ChartAssignmentHasStatus(&ca, crcapps.ChartAssignmentPhaseFailed),
		backoff.WithMaxRetries(backoff.NewConstantBackOff(time.Second), 60),
	); err != nil {
		t.Fatalf("wait for chart assignment to fail: %s", err)
	}
}

func testUpdateChartAssignment_WithFixedDeployment_BecomesReady(t *testing.T, f *kubetest.Fixture) {
	robot := f.Client(robotClusterName)

	// First, create a bad ChartAssignment and wait for it to fail.
	data := map[string]string{
		"cluster":   robotClusterName,
		"name":      f.Uniq("example"),
		"namespace": f.Uniq("ns"),
		"chart":     kubetest.BuildInlineChart(t, "example", deploymentWithBadLabels /*values=*/, ""),
	}
	var ca crcapps.ChartAssignment
	f.FromYAML(inlineChartTemplate, data, &ca)

	if err := robot.Create(f.Ctx(), &ca); err != nil {
		t.Fatalf("create ChartAssignment: %s", err)
	}

	if err := backoff.Retry(
		f.ChartAssignmentHasStatus(&ca, crcapps.ChartAssignmentPhaseFailed),
		backoff.WithMaxRetries(backoff.NewConstantBackOff(time.Second), 60),
	); err != nil {
		t.Fatalf("wait for chart assignment to fail: %s", err)
	}

	// Next, fix the ChartAssignment and wait for it to become ready. We have
	// to retry the read-modify-write in case the controller updates the status
	// in parallel.
	if err := backoff.Retry(func() error {
		if err := robot.Get(f.Ctx(), f.ObjectKey(&ca), &ca); err != nil {
			return backoff.Permanent(err)
		}
		ca.Spec.Chart.Inline = kubetest.BuildInlineChart(t, "example", goodDeployment /*values=*/, "")
		if err := robot.Update(f.Ctx(), &ca); apierrors.IsConflict(err) {
			return err
		} else if err != nil {
			return backoff.Permanent(err)
		}
		return nil
	}, backoff.WithMaxRetries(backoff.NewConstantBackOff(time.Second), 60),
	); err != nil {
		t.Fatalf("update ChartAssignment: %s %s", apierrors.ReasonForError(err), err)
	}

	if err := backoff.Retry(
		f.ChartAssignmentHasStatus(&ca, crcapps.ChartAssignmentPhaseReady),
		backoff.WithMaxRetries(backoff.NewConstantBackOff(time.Second), 60),
	); err != nil {
		t.Fatalf("wait for chart assignment to go from Failed to Ready: %s", err)
	}
}

func testUpdateChartAssignment_WithFixedJob_BecomesReady(t *testing.T, f *kubetest.Fixture) {
	robot := f.Client(robotClusterName)

	// First, create a ChartAssignment with a bad job and wait for it to settle.
	data := map[string]string{
		"cluster":   robotClusterName,
		"name":      f.Uniq("example"),
		"namespace": f.Uniq("ns"),
		"chart":     kubetest.BuildInlineChart(t, "example", badJob /*values=*/, ""),
	}
	var ca crcapps.ChartAssignment
	f.FromYAML(inlineChartTemplate, data, &ca)

	if err := robot.Create(f.Ctx(), &ca); err != nil {
		t.Fatalf("create ChartAssignment: %s", err)
	}

	if err := backoff.Retry(
		f.ChartAssignmentHasStatus(&ca, crcapps.ChartAssignmentPhaseSettled),
		backoff.WithMaxRetries(backoff.NewConstantBackOff(time.Second), 60),
	); err != nil {
		t.Fatalf("wait for chart assignment to settle: %s", err)
	}

	// Next, change the ChartAssignment and wait for it to become ready. We have
	// to retry the read-modify-write in case the controller updates the status
	// in parallel.
	if err := backoff.Retry(func() error {
		if err := robot.Get(f.Ctx(), f.ObjectKey(&ca), &ca); err != nil {
			return backoff.Permanent(err)
		}
		ca.Spec.Chart.Inline = kubetest.BuildInlineChart(t, "example", goodJob /*values=*/, "")
		if err := robot.Update(f.Ctx(), &ca); apierrors.IsConflict(err) {
			return err
		} else if err != nil {
			return backoff.Permanent(err)
		}
		return nil
	}, backoff.WithMaxRetries(backoff.NewConstantBackOff(time.Second), 60),
	); err != nil {
		t.Fatalf("update ChartAssignment: %s %s", apierrors.ReasonForError(err), err)
	}

	if err := backoff.Retry(
		f.ChartAssignmentHasStatus(&ca, crcapps.ChartAssignmentPhaseReady),
		backoff.WithMaxRetries(backoff.NewConstantBackOff(3*time.Second), 60),
	); err != nil {
		t.Fatalf("wait for chart assignment to go from Settled to Ready: %s", err)
	}
}
