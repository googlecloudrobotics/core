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

	cas, err := generateChartAssignments(&app, &rollout, robots[:], baseValues)
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

	cas, err := generateChartAssignments(&app, &rollout, robots[:], nil)
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

	_, err := generateChartAssignments(&app, &rollout, robots[:], nil)
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

func TestValidate(t *testing.T) {
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

			err := validate(&cur)
			if err == nil && c.shouldFail {
				t.Fatal("expected failure but got none")
			}
			if err != nil && !c.shouldFail {
				t.Fatalf("unexpected error: %s", err)
			}
		})
	}
}
