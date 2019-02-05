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

package helm

import (
	"context"
	"io/ioutil"
	"os"
	"path/filepath"
	pb "src/proto/registry"
	"testing"

	. "github.com/onsi/gomega"
	. "github.com/onsi/gomega/gstruct"
	yaml "gopkg.in/yaml.v2"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/client-go/dynamic"
	"k8s.io/client-go/dynamic/fake"
)

func getContentsOrError(t *testing.T, dir string, filename string) string {
	b, err := ioutil.ReadFile(filepath.Join(dir, filename))
	if err != nil {
		t.Errorf("Unable to read %s: %v", filename, err)
	}
	return string(b)
}

var robots = []unstructured.Unstructured{
	robotWithNameAndTypeAndRole("wall-e", "Freight", "Nav (simulated)"),
	robotWithNameAndTypeAndRole("terminator", "Freight", "SLAM"),
}

func fakeKubernetesClientWithCustomResources(customResources ...*unstructured.Unstructured) dynamic.Interface {
	objects := make([]runtime.Object, len(customResources))
	for i, customResource := range customResources {
		objects[i] = customResource
	}
	return fake.NewSimpleDynamicClient(runtime.NewScheme(), objects...)
}

func newHelmWithCustomResources(t *testing.T, customResources ...*unstructured.Unstructured) *Helm {
	client := fakeKubernetesClientWithCustomResources(customResources...)

	helm, err := NewHelm(client, nil)
	if err != nil {
		t.Errorf("NewHelm failed: %v", err)
	}

	return helm
}

func newHelmWithDefaults(t *testing.T) *Helm {
	slamRole := roleWithNamesAndApps(
		"slam",
		"SLAM",
		roleAppWithNameAndValues("freight", "app:\n  role: slam\n  sim_suffix: ''\n"))

	navSimRole := roleWithNamesAndApps(
		"nav-sim",
		"Nav (simulated)",
		roleAppWithNameAndValues("freight", "app:\n  role: nav\n  sim_suffix: -sim\n"))

	freightApp := appWithNameAndCharts(
		"freight",
		chartWithNameAndTarget("freight-cloud", "CLOUD_PER_ROBOT"),
		chartWithNameAndTargetAndValues("freight-robot", "ROBOT", "cloud: false"),
		chartWithNameAndTargetAndValues("cloud-chart", "CLOUD", "cloud: true"))

	return newHelmWithCustomResources(t, slamRole, navSimRole, freightApp)
}

func TestMerge(t *testing.T) {
	g := NewGomegaWithT(t)
	result := merge(
		map[string]interface{}{
			"a": map[string]interface{}{
				"A": 5,
				"B": map[string]interface{}{
					"0": "bar",
					"1": 5,
				},
			},
			"b": 5,
		},
		map[string]interface{}{
			"a": map[string]interface{}{
				"A": map[string]interface{}{
					"0": "baz",
				},
				"B": map[string]interface{}{
					"0": "bar",
					"1": "typechange",
					"2": 6,
				},
			},
		})
	g.Expect(result).To(Equal(map[string]interface{}{
		"a": map[string]interface{}{
			"A": map[string]interface{}{
				"0": "baz",
			},
			"B": map[string]interface{}{
				"0": "bar",
				"1": "typechange",
				"2": 6,
			},
		},
		"b": 5,
	}))
}

func TestBuildPerRobotChart(t *testing.T) {
	g := NewGomegaWithT(t)

	wantRequirements := requirements{
		Dependencies: []dependency{
			{
				Name:    "freight-robot",
				Version: "0.0.1",
				Alias:   "robot-wall-e-app-freight",
			},
			{
				Name:    "freight-robot",
				Version: "0.0.1",
				Alias:   "robot-terminator-app-freight",
			},
		},
	}
	wantValues := `
robot-wall-e-app-freight:
  app:
    role: nav
    sim_suffix: -sim
  cloud: false
  domain: https://cloudrobotics.com
  project: foo-project
  robot:
    name: wall-e
    role: "Nav (simulated)"
    type: Freight
robot-terminator-app-freight:
  app:
    role: slam
    sim_suffix: ""
  cloud: false
  domain: https://cloudrobotics.com
  project: foo-project
  robot:
    name: terminator
    role: SLAM
    type: Freight
`
	helm := newHelmWithDefaults(t)
	helm.helmParams = map[string]interface{}{
		"project": "foo-project",
		"domain":  "https://cloudrobotics.com",
	}

	got, err := helm.buildPerRobotChart(robots, pb.InstallationTarget_ROBOT)
	if err != nil {
		t.Errorf("buildPerRobotChart failed: %v", err)
	}
	g.Expect(got.requirements).To(Equal(wantRequirements))
	valuesYaml, err := yaml.Marshal(got.values)
	g.Expect(valuesYaml).To(MatchYAML(wantValues))
}

func TestBuildPerRobotChartIncludesIfReferencedInRole(t *testing.T) {
	g := NewGomegaWithT(t)
	helm := newHelmWithCustomResources(
		t,
		roleWithNamesAndApps("a-role", "A role", roleAppWithName("my-app")),
		appWithNameAndCharts("my-app", chartWithNameAndTarget("my-chart", "ROBOT")))
	robots := []unstructured.Unstructured{robotWithNameAndRole("A robot", "A role")}

	masterChart, err := helm.buildPerRobotChart(robots, pb.InstallationTarget_ROBOT)
	if err != nil {
		t.Errorf("buildPerRobotChart failed: %v", err)
	}

	g.Expect(masterChart.requirements.Dependencies).To(
		ConsistOf(MatchFields(IgnoreExtras, Fields{"Name": Equal("my-chart")})))
}

func TestBuildPerRobotChartSkipsNonExistingAppIfReferencedInRole(t *testing.T) {
	g := NewGomegaWithT(t)
	helm := newHelmWithCustomResources(
		t,
		roleWithNamesAndApps("a-role", "A role", roleAppWithName("non-existing app")))
	robots := []unstructured.Unstructured{robotWithNameAndRole("A robot", "A role")}

	masterChart, err := helm.buildPerRobotChart(robots, pb.InstallationTarget_ROBOT)
	if err != nil {
		t.Errorf("buildPerRobotChart failed: %v", err)
	}

	g.Expect(masterChart.requirements.Dependencies).To(BeEmpty())
}

func TestCloudMasterChart(t *testing.T) {
	g := NewGomegaWithT(t)

	want := &masterChart{
		requirements: requirements{
			Dependencies: []dependency{
				{
					Name:    "cloud-chart",
					Version: "0.0.1",
					Alias:   "freight",
				},
			},
		},
		values: map[string]interface{}{
			"freight": map[string]interface{}{
				"cloud":   true,
				"domain":  "https://cloudrobotics.com",
				"project": "foo-project",
			},
		},
	}
	helm := newHelmWithDefaults(t)
	helm.helmParams = map[string]interface{}{
		"project": "foo-project",
		"domain":  "https://cloudrobotics.com",
	}

	got, err := helm.buildCloudChart()
	if err != nil {
		t.Errorf("buildPerCloudChart failed: %v", err)
	}
	g.Expect(got.requirements.Dependencies).To(ConsistOf(want.requirements.Dependencies))
	g.Expect(got.values).To(Equal(want.values))
}

func TestSetUpChart(t *testing.T) {
	dir, err := ioutil.TempDir("", "synthetic-chart")
	if err != nil {
		t.Errorf("unable to create temp dir: %v", err)
	}
	defer os.RemoveAll(dir)

	mc := &masterChart{
		requirements: requirements{
			Dependencies: []dependency{
				{
					Name:    "freight-robot",
					Version: "0.0.1",
					Alias:   "robot-wall-e-app-freight",
				},
			},
		},
		values: map[string]interface{}{
			"robot-wall-e-app-freight": map[string]interface{}{
				"robot": map[string]interface{}{
					"id":   "wall-e",
					"name": "wall-e",
					"type": "Freight",
					"role": "Nav (simulated)",
				},
				"app": map[string]interface{}{
					"role":       "nav",
					"sim_suffix": "-sim",
				},
				"cloud": true,
			},
		},
	}

	err = setUpChart(dir, mc)
	if err != nil {
		t.Errorf("setUpChart failed: %v", err)
	}

	expReqs := `dependencies:
- name: freight-robot
  version: 0.0.1
  alias: robot-wall-e-app-freight
`
	if got, want := getContentsOrError(t, dir, "requirements.yaml"), expReqs; got != want {
		t.Errorf("Wrong requirements.yaml; got %s; want %s", got, want)
	}

	expValues := `robot-wall-e-app-freight:
  app:
    role: nav
    sim_suffix: -sim
  cloud: true
  robot:
    id: wall-e
    name: wall-e
    role: Nav (simulated)
    type: Freight
`
	if got, want := getContentsOrError(t, dir, "values.yaml"), expValues; got != want {
		t.Errorf("Wrong values.yaml; got %s; want %s", got, want)
	}

	// Just check that templates/helm-status.yaml was created
	getContentsOrError(t, filepath.Join(dir, "templates"), "helm-status.yaml")

	// Just check that Chart.yaml was created
	getContentsOrError(t, dir, "Chart.yaml")
}

func TestSuccessfulRobotInstall(t *testing.T) {
	ctx := context.Background()
	helm := newHelmWithDefaults(t)
	helm.helmBinary = "/bin/true"

	if err := helm.InstallApps(ctx, "foobar", "default", pb.InstallationTarget_ROBOT, robots); err != nil {
		t.Errorf("helm failed: %v", err)
	}
}

func TestSuccessfulCloudInstall(t *testing.T) {
	ctx := context.Background()
	helm := newHelmWithDefaults(t)
	helm.helmBinary = "/bin/true"

	if err := helm.InstallApps(ctx, "foobar", "default", pb.InstallationTarget_CLOUD, robots); err != nil {
		t.Errorf("helm failed: %v", err)
	}
}

func TestElideCopy(t *testing.T) {
	ctx := context.Background()
	dir, err := ioutil.TempDir("", "synthetic-chart")
	if err != nil {
		t.Errorf("unable to create temp dir: %v", err)
	}
	defer os.RemoveAll(dir)
	helm := newHelmWithDefaults(t)
	helm.helmBinary = "/bin/true"

	if err := helm.InstallApps(ctx, "foobar", "default", pb.InstallationTarget_ROBOT, robots); err != nil {
		t.Errorf("InstallApps failed in first try: %v", err)
	}

	helm.helmBinary = "/bin/false"
	// The only way for this call to succeed is to exit before /bin/false
	// is actually called.
	if err := helm.InstallApps(ctx, "foobar", "default", pb.InstallationTarget_ROBOT, robots); err != nil {
		t.Errorf("InstallApps failed in second try: %v", err)
	}
}

func robotWithNameAndRole(name string, roleDisplayName string) unstructured.Unstructured {
	return robotWithNameAndTypeAndRole(name, "Robot Type", roleDisplayName)
}

func robotWithNameAndTypeAndRole(name string, robotType string, roleDisplayName string) unstructured.Unstructured {
	return unstructured.Unstructured{
		Object: map[string]interface{}{
			"apiVersion": "registry.cloudrobotics.com/v1alpha1",
			"kind":       "Robot",
			"metadata": map[string]interface{}{
				"namespace": "default",
				"name":      name,
				"labels": map[string]interface{}{
					"cloudrobotics.com/robot-name": name,
				},
			},
			"spec": map[string]interface{}{
				"type": robotType,
				"role": roleDisplayName,
			},
		},
	}
}

func roleWithNamesAndApps(name string, displayName string, apps ...interface{}) *unstructured.Unstructured {
	return &unstructured.Unstructured{
		Object: map[string]interface{}{
			"apiVersion": "registry.cloudrobotics.com/v1alpha1",
			"kind":       "Role",
			"metadata": map[string]interface{}{
				"namespace": "default",
				"name":      name,
			},
			"spec": map[string]interface{}{
				"displayName": displayName,
				"apps":        apps,
			},
		},
	}
}

func roleAppWithName(appName string) interface{} {
	return map[string]interface{}{"app": appName}
}

func roleAppWithNameAndValues(appName string, values string) interface{} {
	return map[string]interface{}{"app": appName, "values": values}
}

func appWithNameAndCharts(name string, charts ...interface{}) *unstructured.Unstructured {
	return &unstructured.Unstructured{
		Object: map[string]interface{}{
			"apiVersion": "registry.cloudrobotics.com/v1alpha1",
			"kind":       "App",
			"metadata": map[string]interface{}{
				"namespace": "default",
				"name":      name,
			},
			"spec": map[string]interface{}{
				"charts": charts,
			},
		},
	}
}

func chartWithNameAndTarget(name string, installationTarget string) interface{} {
	return map[string]interface{}{
		"name":                name,
		"version":             "0.0.1",
		"installation_target": installationTarget,
		"inline_chart":        "Zm9vCg==",
	}
}

func chartWithNameAndTargetAndValues(name string, installationTarget string, values string) interface{} {
	chart := chartWithNameAndTarget(name, installationTarget).(map[string]interface{})
	chart["values"] = values
	return chart
}
