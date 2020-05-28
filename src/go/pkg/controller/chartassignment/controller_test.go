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

package chartassignment

import (
	"strings"
	"testing"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"sigs.k8s.io/yaml"
)

func unmarshalYAML(t *testing.T, v interface{}, s string) {
	t.Helper()
	if err := yaml.Unmarshal([]byte(strings.TrimSpace(s)), v); err != nil {
		t.Fatal(err)
	}
}

func TestValidate(t *testing.T) {
	cases := []struct {
		name       string
		old        string
		cur        string
		shouldFail bool
	}{
		{
			name: "valid-with-inline-chart",
			cur: `
spec:
  clusterName: c1
  namespaceName: ns1
  chart:
    inline: abc
    values:
      a: 2
      b: {c: 3}
	`,
		},
		{
			name: "valid-with-reference-chart",
			cur: `
spec:
  clusterName: c1
  namespaceName: ns1
  chart:
    repository: https://some.repo
    name: chartname
    version: 1.3.4
    values:
      a: 2
      b: {c: 3}
	`,
		},
		{
			name: "missing-cluster-name",
			cur: `
spec:
  namespaceName: ns1
  chart:
    inline: abc
	`,
			shouldFail: true,
		},
		{
			name: "missing-namespace-name",
			cur: `
spec:
  clusterName: c1
  chart:
    inline: abc
	`,
			shouldFail: true,
		},
		{
			name: "invalid-cluster-name",
			cur: `
spec:
  clusterName: c1%2
  namespaceName: ns1
  chart:
    inline: abc
	`,
			shouldFail: true,
		},
		{
			name: "invalid-namespace-name",
			cur: `
spec:
  clusterName: c1
  namespaceName: ns1%2
  chart:
    inline: abc
	`,
			shouldFail: true,
		},
		{
			name: "invalid-partial-reference",
			cur: `
spec:
  clusterName: c1
  namespaceName: ns1%2
  chart:
    name: chartname
    version: 1.3.4
	`,
			shouldFail: true,
		},
		{
			name: "invalid-inline-and-reference-chart",
			cur: `
spec:
  clusterName: c1
  namespaceName: ns1%2
  chart:
    inline: abc
    repository: https://some.repo
    name: chartname
    version: 1.3.4
	`,
			shouldFail: true,
		},
		{
			name: "cluster-name-changed",
			old: `
spec:
  clusterName: c1
  namespaceName: ns1
  chart:
    inline: abc
	`,
			cur: `
spec:
  clusterName: c2
  namespaceName: ns1
  chart:
    inline: abc
	`,
			shouldFail: true,
		},
		{
			name: "namespace-name-changed",
			old: `
spec:
  clusterName: c1
  namespaceName: ns1
  chart:
    inline: abc
	`,
			cur: `
spec:
  clusterName: c1
  namespaceName: ns2
  chart:
    inline: abc
	`,
			shouldFail: true,
		},
	}
	for _, c := range cases {
		v := newChartAssignmentValidator(nil)
		t.Run(c.name, func(t *testing.T) {
			var old, cur *apps.ChartAssignment

			if c.old != "" {
				old = &apps.ChartAssignment{}
				unmarshalYAML(t, &old, c.old)
			}
			if c.cur != "" {
				cur = &apps.ChartAssignment{}
				unmarshalYAML(t, &cur, c.cur)
			}
			err := v.validate(cur, old)
			if err == nil && c.shouldFail {
				t.Fatal("expected failure but got none")
			}
			if err != nil && !c.shouldFail {
				t.Fatalf("unexpected error: %s", err)
			}
		})
	}
}

func TestValidateForOnPremCluster(t *testing.T) {
	cases := []struct {
		name       string
		old        string
		cur        string
		shouldFail bool
	}{
		{
			name: "valid-with-correct-name",
			cur: `
spec:
  clusterName: my-cluster
  namespaceName: ns1
  chart:
    inline: abc
	`,
		},
		{
			name: "valid-with-incorrect-name",
			cur: `
spec:
  clusterName: not-my-cluster
  namespaceName: ns1
  chart:
    inline: abc
	`,
			shouldFail: true,
		},
	}
	for _, c := range cases {
		v := newChartAssignmentValidator(nil)
		v.clusterName = "my-cluster"
		t.Run(c.name, func(t *testing.T) {
			cur := &apps.ChartAssignment{}
			unmarshalYAML(t, &cur, c.cur)
			err := v.validate(cur, nil)
			if err == nil && c.shouldFail {
				t.Fatal("expected failure but got none")
			}
			if err != nil && !c.shouldFail {
				t.Fatalf("unexpected error: %s", err)
			}
		})
	}
}
