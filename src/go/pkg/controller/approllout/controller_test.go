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

package approllout

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
