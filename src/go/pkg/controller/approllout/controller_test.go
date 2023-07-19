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

package approllout

import (
	"strings"
	"testing"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	registry "github.com/googlecloudrobotics/core/src/go/pkg/apis/registry/v1alpha1"
	core "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/helm/pkg/chartutil"
	"sigs.k8s.io/yaml"
)

func marshalYAML(t *testing.T, v interface{}) string {
	t.Helper()
	b, err := yaml.Marshal(v)
	if err != nil {
		t.Fatal(err)
	}
	return string(b)
}

func unmarshalYAML(t *testing.T, v interface{}, s string) {
	t.Helper()
	if err := yaml.Unmarshal([]byte(strings.TrimSpace(s)), v); err != nil {
		t.Fatal(err)
	}
}

func verifyChartAssignment(t *testing.T, want, got *apps.ChartAssignment) {
	t.Helper()
	// Compare serialized YAML for easier diff detection and to avoid complicated
	// comparisons for map[string]interface{} values.
	wantStr := marshalYAML(t, want)
	gotStr := marshalYAML(t, got)

	if wantStr != gotStr {
		t.Fatalf("expected ChartAssignment: \n%s\ngot:\n%s\n", wantStr, gotStr)
	}
}

func TestNewRobotChartAssignment(t *testing.T) {
	var app apps.App
	unmarshalYAML(t, &app, `
metadata:
  name: foo
spec:
  repository: https://example.org/helm
  version: 1.2.3
  components:
    robot:
      name: foo-robot
      inline: abcdefgh
	`)

	var rollout apps.AppRollout
	unmarshalYAML(t, &rollout, `
metadata:
  name: foo-rollout
  labels:
    lkey1: lval1
  annotations:
    akey1: aval1
  namespace: default
spec:
  appName: prometheus
  robots:
  - selector:
      any: true
    values:
      foo1: bar1
    version: 1.2.4
 `)

	var robot registry.Robot
	unmarshalYAML(t, &robot, `
metadata:
  name: robot1
	`)

	baseValues := chartutil.Values{
		"foo2": "bar2",
	}

	var expected apps.ChartAssignment
	unmarshalYAML(t, &expected, `
metadata:
  name: foo-rollout-robot-robot1
  labels:
    lkey1: lval1
    cloudrobotics.com/robot-name: robot1
  annotations:
    akey1: aval1
spec:
  clusterName: robot1
  namespaceName: app-foo-rollout
  chart:
    repository: https://example.org/helm
    version: 1.2.4
    name: foo-robot
    inline: abcdefgh
    values:
      robot:
        name: robot1
      foo1: bar1
      foo2: bar2
	`)

	result := newRobotChartAssignment(&robot, &app, &rollout, &rollout.Spec.Robots[0], baseValues)
	verifyChartAssignment(t, &expected, result)
}

func TestNewCloudChartAssignment(t *testing.T) {
	var app apps.App
	unmarshalYAML(t, &app, `
metadata:
  name: foo
spec:
  repository: https://example.org/helm
  version: 1.2.3
  components:
    cloud:
      name: foo-cloud
      inline: abcdefgh
	`)

	var rollout apps.AppRollout
	unmarshalYAML(t, &rollout, `
metadata:
  name: foo-rollout
  labels:
    lkey1: lval1
  annotations:
    akey1: aval1
  namespace: default
spec:
  appName: prometheus
  cloud:
    values:
      robots: should_be_overwritten
      foo1: bar1
 `)

	var robot1, robot2 registry.Robot
	unmarshalYAML(t, &robot1, `
metadata:
  name: robot1
	`)
	unmarshalYAML(t, &robot2, `
metadata:
  name: robot2
	`)

	baseValues := chartutil.Values{
		"foo2": "bar2",
	}

	var expected apps.ChartAssignment
	unmarshalYAML(t, &expected, `
metadata:
  name: foo-rollout-cloud
  labels:
    lkey1: lval1
  annotations:
    akey1: aval1
spec:
  clusterName: cloud
  namespaceName: app-foo-rollout
  chart:
    repository: https://example.org/helm
    version: 1.2.3
    name: foo-cloud
    inline: abcdefgh
    values:
      robots:
      - name: robot1
      - name: robot2
      foo1: bar1
      foo2: bar2
	`)

	result := newCloudChartAssignment(&app, &rollout, baseValues, &robot1, &robot2)
	verifyChartAssignment(t, &expected, result)
}

func TestGenerateChartAssignments(t *testing.T) {
	var app apps.App
	unmarshalYAML(t, &app, `
metadata:
  name: foo
spec:
  components:
    cloud:
      inline: inline-cloud
    robot:
      inline: inline-robot
	`)

	var robots [3]registry.Robot

	unmarshalYAML(t, &robots[0], `
metadata:
  name: robot1
	`)
	unmarshalYAML(t, &robots[1], `
metadata:
  name: robot2
  labels:
    a: b
	`)
	unmarshalYAML(t, &robots[2], `
metadata:
  name: robot3
  labels:
    a: c
	`)

	baseValues := chartutil.Values{
		"foo2": "bar2",
	}

	// Rollout with two selectors that select robot1 and robot3 respectively.
	// robot2 is not matched at all.
	var rollout apps.AppRollout
	unmarshalYAML(t, &rollout, `
metadata:
  name: foo-rollout
  namespace: default
spec:
  appName: foo
  cloud:
    values:
      robots: should_be_overwritten
      foo1: bar1
  robots:
  # robot1
  - selector:
      matchExpressions:
      - {key: a, operator: DoesNotExist}
  # robot3
  - selector:
      matchLabels:
        a: c
    values:
      foo3: bar3
	`)

	var expected [3]apps.ChartAssignment
	unmarshalYAML(t, &expected[0], `
metadata:
  name: foo-rollout-cloud
spec:
  clusterName: cloud
  namespaceName: app-foo-rollout
  chart:
    inline: inline-cloud
    values:
      foo1: bar1
      foo2: bar2
      robots:
      - name: robot1
      - name: robot3
	`)
	unmarshalYAML(t, &expected[1], `
metadata:
  name: foo-rollout-robot-robot1
  labels:
    cloudrobotics.com/robot-name: robot1
spec:
  clusterName: robot1
  namespaceName: app-foo-rollout
  chart:
    inline: inline-robot
    values:
      robot:
        name: robot1
      foo2: bar2
      `)
	unmarshalYAML(t, &expected[2], `
metadata:
  name: foo-rollout-robot-robot3
  labels:
    cloudrobotics.com/robot-name: robot3
spec:
  clusterName: robot3
  namespaceName: app-foo-rollout
  chart:
    inline: inline-robot
    values:
      robot:
        name: robot3
      foo2: bar2
      foo3: bar3
      `)

	al := []apps.App{app}
	cas, err := generateChartAssignments(al, robots[:], &rollout, baseValues)
	if err != nil {
		t.Fatalf("Generate failed: %s", err)
	}
	if len(cas) != len(expected) {
		t.Errorf("Expected %d ChartAssignments, got %d", len(expected), len(cas))
	}
	for i, ca := range cas {
		verifyChartAssignment(t, &expected[i], ca)
	}
}

// generateApp will generate an app for testing
func generateApp(name, version, robotPayload, cloudPayload string) apps.App {
	app := apps.App{}
	app.Name = name
	app.Labels = map[string]string{
		labelAppName:    name,
		labelAppVersion: version,
	}
	app.Spec.Components.Cloud.Name = "cloud"
	app.Spec.Components.Cloud.Inline = cloudPayload
	app.Spec.Components.Robot.Name = "robot"
	app.Spec.Components.Robot.Inline = robotPayload
	return app
}

// generateRobot will generate a robot named |name| with the labels
func generateRobot(name string, labels map[string]string) registry.Robot {
	robot := registry.Robot{}
	robot.Name = name
	robot.Labels = labels
	return robot
}

// generateRollout will create a rollout pointing at AppName
func generateRollout(name, appName string) apps.AppRollout {
	rollout := apps.AppRollout{}
	rollout.Name = name
	rollout.Spec.AppName = appName
	return rollout
}

// addRobotToRollout will add a robot component with the match label & version.
func addRobotToRollout(ar *apps.AppRollout, matchLabel, matchValue, version string) {
	ar.Spec.Robots = append(ar.Spec.Robots, apps.AppRolloutSpecRobot{
		Version: version,
		Selector: &apps.RobotSelector{
			LabelSelector: &metav1.LabelSelector{
				MatchLabels: map[string]string{
					matchLabel: matchValue,
				},
			},
		},
	})
}

// The tests below deal with vApp, this is a short form for versioned App

// TestGenerateChartAssignments_vApps tests rollout with versioned Apps
func TestGenerateChartAssignments_vApps(t *testing.T) {
	appName := "myapp"
	appVersion := "17"
	testLabel := "test-label"
	testValueA := "test-value"
	testValueB := "test-value-2"
	al := []apps.App{
		generateApp(appName, "", "testpayload", ""),
		generateApp(appName, appVersion, "testpayload", ""),
	}
	robots := []registry.Robot{
		generateRobot("robot1", map[string]string{
			testLabel: testValueA,
		}),
		generateRobot("robot2", map[string]string{
			testLabel: testValueA,
		}),
		generateRobot("robot3", map[string]string{
			testLabel: testValueB,
		}),
	}
	rollout := generateRollout("test", appName)
	// a versioned app matching robots testValueA
	addRobotToRollout(&rollout, testLabel, testValueA, appVersion)
	// the canonical app matching robots testValueB
	addRobotToRollout(&rollout, testLabel, testValueB, "")
	// Simply test that this works, and does not throw errors.
	_, err := generateChartAssignments(al, robots[:], &rollout, nil)
	if err != nil {
		t.Fatalf("Generate failed: %v", err)
	}
}

// TestGenerateChartAssignments_vAppMissing tests a missing App version
func TestGenerateChartAssignments_vAppMissing(t *testing.T) {
	appName := "myapp"
	appVersion := "17"
	newAppVersion := "18"
	testLabel := "test-label"
	testValueA := "test-value"
	al := []apps.App{
		generateApp(appName, appVersion, "testpayload", ""),
	}
	robots := []registry.Robot{
		generateRobot("robot1", map[string]string{
			testLabel: testValueA,
		}),
	}
	rollout := generateRollout("test", appName)
	// a versioned app matching robots testValueA
	addRobotToRollout(&rollout, testLabel, testValueA, newAppVersion)
	// Simply test that this works, and does throw an error.
	_, err := generateChartAssignments(al, robots[:], &rollout, nil)
	if err == nil {
		t.Fatal("Requesting a non-existant app should fail, but didn't")
	}
}

// TestGenerateChartAssignments_vAppCloud tests a cloud cas
func TestGenerateChartAssignments_vAppCloud(t *testing.T) {
	appName := "myapp"
	appVersion := "17"
	testLabel := "test-label"
	testValueA := "test-value"
	al := []apps.App{
		// Cloud does not have a Version field, and thus requires this
		// non versioned, canonical app.
		generateApp(appName, "", "testpayload", "cloudpayload"),
		generateApp(appName, appVersion, "testpayload", ""),
	}
	robots := []registry.Robot{
		generateRobot("robot1", map[string]string{
			testLabel: testValueA,
		}),
	}
	rollout := generateRollout("test", appName)
	// a versioned app matching robots testValueA
	addRobotToRollout(&rollout, testLabel, testValueA, appVersion)
	// expand to cover cloud
	rollout.Spec.Cloud = apps.AppRolloutSpecCloud{}
	// Simply test that this works, and does throw an error.
	cas, err := generateChartAssignments(al, robots[:], &rollout, nil)
	if err != nil {
		t.Fatal("generate chart assignment with cloud rollout")
	}
	if len(cas) != 2 {
		t.Fatalf("chart assingments, expected 2, got %d", len(cas))
	}
}

// TestGenerateChartAssignments_vAppCloudMissing tests cloud rollout w/out App
//
// As the Cloud field is not versioned, it needs the canonical version of the
// app to exist. In this test, we only create versioned Apps, and thus
// the generation will fail.
func TestGenerateChartAssignments_vAppCloudMissing(t *testing.T) {
	appName := "myapp"
	appVersion := "17"
	al := []apps.App{
		// only the versioned App will exist.
		generateApp(appName, appVersion, "testpayload", "cloudpayload"),
	}
	rollout := generateRollout("test", appName)
	// expand to cover cloud
	rollout.Spec.Cloud = apps.AppRolloutSpecCloud{}
	// Simply test that this works, and does throw an error.
	_, err := generateChartAssignments(al, nil, &rollout, nil)
	if err != nil {
		t.Fatal("cloud rollout without unversioned app should pass, if rollout has no Cloud Values in Spec.")
	}
	rollout.Spec.Cloud = apps.AppRolloutSpecCloud{Values: apps.ConfigValues{
		"test": 1,
	}}
	// Simply test that this works, and does throw an error.
	_, err = generateChartAssignments(al, nil, &rollout, nil)
	if err != nil {
		t.Fatal("cloud rollout without unversioned app should pass, (and log) if rollout has Cloud Values in Spec.")
	}
}

func TestGenerateChartAssignments_cloudPerRobot(t *testing.T) {
	var app apps.App
	unmarshalYAML(t, &app, `
metadata:
  name: foo
spec:
  components:
    cloud:
      inline: inline-cloud
	`)

	var robots [2]registry.Robot

	unmarshalYAML(t, &robots[0], `
metadata:
  name: robot1
	`)
	unmarshalYAML(t, &robots[1], `
metadata:
  name: robot2
  labels:
    a: b
	`)

	// Rollout selects robot1, but not robot2.
	var rollout apps.AppRollout
	unmarshalYAML(t, &rollout, `
metadata:
  name: foo-rollout
  namespace: default
spec:
  appName: foo
  cloud:
    values:
      robots: should_be_overwritten
  robots:
  # robot1
  - selector:
      matchExpressions:
      - {key: a, operator: DoesNotExist}
	`)

	var expected apps.ChartAssignment
	unmarshalYAML(t, &expected, `
metadata:
  name: foo-rollout-cloud
spec:
  clusterName: cloud
  namespaceName: app-foo-rollout
  chart:
    inline: inline-cloud
    values:
      robots:
      - name: robot1
	`)

	al := []apps.App{app}
	cas, err := generateChartAssignments(al, robots[:], &rollout, nil)
	if err != nil {
		t.Fatalf("Generate failed: %s", err)
	}
	if len(cas) != 1 {
		t.Fatalf("Expected 1 ChartAssignments, got %d", len(cas))
	}
	verifyChartAssignment(t, &expected, cas[0])
}

func TestGenerateChartAssignments_selectorOverlap(t *testing.T) {
	var app apps.App
	unmarshalYAML(t, &app, `
metadata:
  name: foo
spec:
  components:
    robot:
      inline: inline-robot
	`)

	var robots [2]registry.Robot
	unmarshalYAML(t, &robots[0], `
metadata:
  name: robot1
	`)
	unmarshalYAML(t, &robots[1], `
metadata:
  name: robot2
  labels:
    a: b
	`)

	// Rollout with two selectors that match the same robot.
	var rollout apps.AppRollout
	unmarshalYAML(t, &rollout, `
metadata:
  name: foo-rollout
spec:
  appName: foo
  robots:
  - selector:
      any: true
  - selector:
      matchLabels:
        a: b
	`)
	al := []apps.App{app}
	_, err := generateChartAssignments(al, robots[:], &rollout, nil)
	if exp := errRobotSelectorOverlap("robot2"); err != exp {
		t.Fatalf("expected error %q but got %q", exp, err)
	}
}

func TestSetStatus(t *testing.T) {
	var ca1, ca2, ca3 apps.ChartAssignment
	unmarshalYAML(t, &ca1, `
metadata:
  name: ca1
status:
  phase: Failed
	`)
	unmarshalYAML(t, &ca2, `
metadata:
  name: ca2
status:
  phase: Settled
	`)
	unmarshalYAML(t, &ca3, `
metadata:
  name: ca3
status:
  phase: Ready
	`)

	var ar apps.AppRollout
	setStatus(&ar, 100, []apps.ChartAssignment{ca1, ca2, ca3})

	if ar.Status.Assignments != 100 {
		t.Errorf("Expected .status.assignments to be %d but got %d", 100, ar.Status.Assignments)
	}
	if ar.Status.FailedAssignments != 1 {
		t.Errorf("Expected .status.failedAssignments to be %d but got %d", 1, ar.Status.FailedAssignments)
	}
	if ar.Status.SettledAssignments != 2 {
		t.Errorf("Expected .status.settledAssignments to be %d but got %d", 2, ar.Status.SettledAssignments)
	}
	if ar.Status.ReadyAssignments != 1 {
		t.Errorf("Expected .status.readyAssignments to be %d but got %d", 1, ar.Status.ReadyAssignments)
	}
	if c := ar.Status.Conditions[0]; c.Type != apps.AppRolloutConditionSettled ||
		c.Status != core.ConditionFalse {
		t.Errorf("Unexpected first condition %v, expected Settled=False", c)
	}
	if c := ar.Status.Conditions[1]; c.Type != apps.AppRolloutConditionReady ||
		c.Status != core.ConditionFalse {
		t.Errorf("Unexpected second condition %v, expected Ready=False", c)
	}
}

func TestValidateAppRollout(t *testing.T) {
	cases := []struct {
		name       string
		cur        string
		shouldFail bool
	}{
		{
			name: "valid-all",
			cur: `
spec:
  appName: myapp
  cloud:
    values:
      a: 2
      b: {c: 3}
  robots:
  - selector:
      any: true
    values:
      c: d
  - selector:
      matchLabels:
        abc: def
        foo: bar
  - selector:
      matchExpressions:
      - {key: foo, Op: DoesExist}
	`,
		},
		{
			name: "valid-app-name-only",
			cur: `
spec:
  appName: my-app.123
	`,
		},
		{
			name:       "missing-app-name",
			cur:        `spec: {}`,
			shouldFail: true,
		},
		{
			name: "invalid-app-name",
			cur: `
spec:
  appName: my%app
	`,
			shouldFail: true,
		},
		{
			name: "missing-robot-selector",
			cur: `
spec:
  appName: myapp
  robots:
  - values:
      a: b
	`,
			shouldFail: true,
		},
		{
			name: "wrong-selector",
			cur: `
spec:
  appName: myapp
  robots:
  - selector:
      a: b
	`,
			shouldFail: true,
		},
		{
			name: "cloud-values-Robots",
			cur: `
spec:
  appName: myapp
  cloud:
    values:
      robots:
        c: d
	`,
			shouldFail: true,
		},
		{
			name: "robot-values-robot",
			cur: `
spec:
  appName: myapp
  robots:
  - selector:
      any: true
    values:
      robot:
        c: d
	`,
			shouldFail: true,
		},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			var cur apps.AppRollout
			unmarshalYAML(t, &cur, c.cur)

			err := appRolloutValidate(&cur)
			if err == nil && c.shouldFail {
				t.Fatal("expected failure but got none")
			}
			if err != nil && !c.shouldFail {
				t.Fatalf("unexpected error: %s", err)
			}
		})
	}
}

// TestValidateApp tests all the labels and mechanism for the apps
func TestValidateApp(t *testing.T) {
	cases := []struct {
		name       string
		cur        string
		shouldFail bool
	}{
		{
			name: "valid",
			cur: `
metadata:
  name: app
	`,
		},
		{
			name: "valid-app-label-only",
			cur: `
metadata:
  name: app
  labels:
    cloudrobotics.com/app-name: app
	`,
		},
		{
			name: "valid-both-labels",
			cur: `
metadata:
  name: app.v17
  labels:
    cloudrobotics.com/app-name: app
    cloudrobotics.com/app-version: 17
	`,
		},
		{
			name: "valid-both-labels-lower",
			cur: `
metadata:
  name: app.v17rc00
  labels:
    cloudrobotics.com/app-name: app
    cloudrobotics.com/app-version: 17RC00
	`,
		},
		{
			name:       "invalid-app-label-only",
			shouldFail: true,
			cur: `
metadata:
  name: app2
  labels:
    cloudrobotics.com/app-name: app
	`,
		},
		{
			name:       "invalid-both-labels",
			shouldFail: true,
			cur: `
metadata:
  name: app.v17rc10
  labels:
    cloudrobotics.com/app-name: app
    cloudrobotics.com/app-version: 17RC00
	`,
		},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			var cur apps.App
			unmarshalYAML(t, &cur, c.cur)

			err := appValidate(&cur)
			if err == nil && c.shouldFail {
				t.Fatal("expected failure but got none")
			}
			if err != nil && !c.shouldFail {
				t.Fatalf("unexpected error: %s", err)
			}
		})
	}
}
