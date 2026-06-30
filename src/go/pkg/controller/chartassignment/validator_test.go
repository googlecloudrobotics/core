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
	"encoding/json"
	"net/http"
	"strings"
	"testing"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	admissionv1 "k8s.io/api/admission/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/runtime"
	"sigs.k8s.io/controller-runtime/pkg/webhook/admission"
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
			name: "missing-namespace-name",
			cur: `
spec:
  chart:
    inline: abc
	`,
			shouldFail: true,
		},
		{
			name: "invalid-namespace-name",
			cur: `
spec:
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
			name: "namespace-name-changed",
			old: `
spec:
  namespaceName: ns1
  chart:
    inline: abc
	`,
			cur: `
spec:
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

func TestHandle(t *testing.T) {
	scheme := runtime.NewScheme()
	if err := apps.AddToScheme(scheme); err != nil {
		t.Fatalf("failed to add apps to scheme: %v", err)
	}
	v := newChartAssignmentValidator(scheme)

	tests := []struct {
		name        string
		req         admission.Request
		wantAllowed bool
		wantCode    int32
	}{
		{
			name: "allowed",
			req: admission.Request{
				AdmissionRequest: admissionv1.AdmissionRequest{
					Object: runtime.RawExtension{
						Raw: mustMarshal(&apps.ChartAssignment{
							ObjectMeta: metav1.ObjectMeta{
								Name: "my-ca",
							},
							Spec: apps.ChartAssignmentSpec{
								NamespaceName: "ns1",
								Chart: apps.AssignedChart{
									Inline: "abc",
								},
							},
						}),
					},
				},
			},
			wantAllowed: true,
		},
		{
			name: "denied_validation",
			req: admission.Request{
				AdmissionRequest: admissionv1.AdmissionRequest{
					Object: runtime.RawExtension{
						Raw: mustMarshal(&apps.ChartAssignment{
							ObjectMeta: metav1.ObjectMeta{
								Name: "my-ca",
							},
							Spec: apps.ChartAssignmentSpec{
								Chart: apps.AssignedChart{
									Inline: "abc",
								},
							},
						}),
					},
				},
			},
			wantAllowed: false,
		},
		{
			name: "decode_error",
			req: admission.Request{
				AdmissionRequest: admissionv1.AdmissionRequest{
					Object: runtime.RawExtension{Raw: []byte("invalid json")},
				},
			},
			wantAllowed: false,
			wantCode:    http.StatusBadRequest,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			resp := v.Handle(t.Context(), tt.req)
			if resp.Allowed != tt.wantAllowed {
				t.Errorf("Handle() Allowed = %v, want %v. Response: %+v", resp.Allowed, tt.wantAllowed, resp)
			}
			if tt.wantCode != 0 && resp.Result.Code != tt.wantCode {
				t.Errorf("Handle() Code = %v, want %v. Response: %+v", resp.Result.Code, tt.wantCode, resp)
			}
		})
	}
}

func mustMarshal(v interface{}) []byte {
	b, err := json.Marshal(v)
	if err != nil {
		panic(err)
	}
	return b
}
