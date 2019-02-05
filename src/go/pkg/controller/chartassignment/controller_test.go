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

package chartassignment

import (
	"encoding/base64"
	"io/ioutil"
	"os"
	"path"
	"strings"
	"testing"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"k8s.io/helm/pkg/chartutil"
	"k8s.io/helm/pkg/helm"
	"sigs.k8s.io/yaml"
)

func unmarshalYAML(t *testing.T, v interface{}, s string) {
	t.Helper()
	if err := yaml.Unmarshal([]byte(strings.TrimSpace(s)), v); err != nil {
		t.Fatal(err)
	}
}

func writeFile(t *testing.T, fn string, s string) {
	t.Helper()
	if err := ioutil.WriteFile(fn, []byte(strings.TrimSpace(s)), 0666); err != nil {
		t.Fatal(err)
	}
}

func buildInlineChart(t *testing.T, chart, values string) string {
	t.Helper()

	tmpdir, err := ioutil.TempDir("", "buildInlineChart")
	if err != nil {
		t.Fatal(err)
	}
	defer os.RemoveAll(tmpdir)

	writeFile(t, path.Join(tmpdir, "Chart.yaml"), chart)
	writeFile(t, path.Join(tmpdir, "values.yaml"), values)

	ch, err := chartutil.LoadDir(tmpdir)
	if err != nil {
		t.Fatal(err)
	}
	fn, err := chartutil.Save(ch, tmpdir)
	if err != nil {
		t.Fatal(err)
	}
	rawChart, err := ioutil.ReadFile(fn)
	if err != nil {
		t.Fatal(err)
	}
	return base64.StdEncoding.EncodeToString(rawChart)
}

func TestReconciler_applyChartOnce(t *testing.T) {
	chart := buildInlineChart(t, `
name: testchart
version: 2.1.0
	`, `
foo1:
  baz1: "hello"
bar1: 3
	`)

	var as apps.ChartAssignment
	unmarshalYAML(t, &as, `
metadata:
  name: test-assignment-1
spec:
  chart:
    values:
      bar2:
        baz2: test
	`)
	as.Spec.Chart.Inline = chart

	r := &Reconciler{
		values: chartutil.Values{
			"bar1": "world",
			"bar2": chartutil.Values{
				"baz2": "never_shown",
			},
		},
		helm: &helm.FakeClient{},
	}
	wantValues := chartutil.Values{
		"bar1": "world",
		"bar2": chartutil.Values{"baz2": "test"},
		"foo1": chartutil.Values{"baz1": "hello"},
	}

	// First apply, the chart should be installed.
	if err := r.applyChart(&as); err != nil {
		t.Fatal(err)
	}
	verifyRelease(t, r.helm, as.Name, 1, wantValues, releaseReasonInstall)
}

func TestReconciler_applyChartTwice(t *testing.T) {
	chart := buildInlineChart(t, `
name: testchart
version: 2.1.0
	`, `
foo1:
  baz1: "hello"
bar1: 3
	`)

	var as apps.ChartAssignment
	unmarshalYAML(t, &as, `
metadata:
  name: test-assignment-1
spec:
  chart:
    values:
      bar2:
        baz2: test
	`)
	as.Spec.Chart.Inline = chart

	r := &Reconciler{
		values: chartutil.Values{
			"bar1": "world",
			"bar2": chartutil.Values{
				"baz2": "never_shown",
			},
		},
		helm: &helm.FakeClient{},
	}

	// First apply, the chart should be installed.
	if err := r.applyChart(&as); err != nil {
		t.Fatal(err)
	}
	// Apply the chart again with changed values should upgrade the
	// existing release.
	as.Spec.Chart.Values["bar3"] = 4

	wantValues := chartutil.Values{
		"bar1": "world",
		"bar2": chartutil.Values{"baz2": "test"},
		"foo1": chartutil.Values{"baz1": "hello"},
		"bar3": 4,
	}
	if err := r.applyChart(&as); err != nil {
		t.Fatal(err)
	}
	verifyRelease(t, r.helm, as.Name, 2, wantValues, releaseReasonUpgrade)
}

func verifyRelease(t *testing.T, helmClient helm.Interface, release string, wantVersion int32, wantValues chartutil.Values, wantReleaseReason releaseReason) {
	resp, err := helmClient.ReleaseContent(release)
	if err != nil {
		t.Fatalf("get release content: %s", err)
	}
	rel := resp.Release

	if rel.Version != wantVersion {
		t.Fatalf("unexpected release revision: want %d, got %d", wantVersion, rel.Version)
	}
	if want, err := wantValues.YAML(); err != nil {
		t.Fatal(err)
	} else if want != rel.Config.Raw {
		t.Fatalf("config values do not match: want\n%s\n\ngot\n%s\n", want, rel.Config.Raw)
	}
	if got := decodeReleaseDesc(rel.Info.Description); got.Reason != wantReleaseReason {
		t.Fatalf("unexpected release reason: want %q, got %q", wantReleaseReason, got.Reason)
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
			err := validate(cur, old)
			if err == nil && c.shouldFail {
				t.Fatal("expected failure but got none")
			}
			if err != nil && !c.shouldFail {
				t.Fatalf("unexpected error: %s", err)
			}
		})
	}
}
