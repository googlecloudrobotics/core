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
	"encoding/base64"
	"io/ioutil"
	"os"
	"path"
	"strings"
	"testing"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"k8s.io/client-go/tools/record"
	"k8s.io/helm/pkg/chartutil"
	"k8s.io/helm/pkg/helm"
)

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

func TestRelease_updateChartOnce(t *testing.T) {
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
      bar1: 4
      bar2:
        baz2: test
	`)
	as.Spec.Chart.Inline = chart

	r := &release{
		helm:     &helm.FakeClient{},
		recorder: &record.FakeRecorder{},
	}
	wantValues := chartutil.Values{
		"bar1": 4,
		"bar2": chartutil.Values{"baz2": "test"},
		"foo1": chartutil.Values{"baz1": "hello"},
	}

	// First apply, the chart should be installed.
	r.updateHelm(&as)
	verifyRelease(t, r.helm, as.Name, 1, wantValues)
}

func TestRelease_updateChartTwice(t *testing.T) {
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
      bar1: 4
      bar2:
        baz2: test
	`)
	as.Spec.Chart.Inline = chart

	r := &release{
		helm:     &helm.FakeClient{},
		recorder: &record.FakeRecorder{},
	}

	// First apply, the chart should be installed.
	r.updateHelm(&as)
	// Apply the chart again with changed values should upgrade the
	// existing release.
	as.Spec.Chart.Values["bar1"] = 5

	wantValues := chartutil.Values{
		"bar1": 5,
		"bar2": chartutil.Values{"baz2": "test"},
		"foo1": chartutil.Values{"baz1": "hello"},
	}
	r.updateHelm(&as)
	verifyRelease(t, r.helm, as.Name, 2, wantValues)
}

func verifyRelease(t *testing.T, helmClient helm.Interface, release string, wantVersion int32, wantValues chartutil.Values) {
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
}
