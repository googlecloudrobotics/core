// Copyright 2026 The Cloud Robotics Authors
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
	"encoding/json"
	"net/http"
	"testing"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	admissionv1 "k8s.io/api/admission/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/runtime/serializer"
	"sigs.k8s.io/controller-runtime/pkg/webhook/admission"
)

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

func TestAppValidator_Handle(t *testing.T) {
	scheme := runtime.NewScheme()
	apps.AddToScheme(scheme)

	decoder := serializer.NewCodecFactory(scheme).UniversalDeserializer()
	v := &appValidator{decoder: decoder}

	tests := []struct {
		name       string
		req        admission.Request
		wantAllow  bool
		wantStatus int32
	}{
		{
			name: "DecodeError",
			req: admission.Request{
				AdmissionRequest: admissionv1.AdmissionRequest{
					Object: runtime.RawExtension{Raw: []byte("invalid json")},
				},
			},
			wantAllow:  false,
			wantStatus: http.StatusBadRequest,
		},
		{
			name: "Allowed",
			req: admission.Request{
				AdmissionRequest: admissionv1.AdmissionRequest{
					Object: runtime.RawExtension{
						Raw: mustMarshal(&apps.App{
							ObjectMeta: metav1.ObjectMeta{
								Name: "myapp",
								Labels: map[string]string{
									labelAppName: "myapp",
								},
							},
						}),
					},
				},
			},
			wantAllow: true,
		},
		{
			name: "Denied_ValidationFailure",
			req: admission.Request{
				AdmissionRequest: admissionv1.AdmissionRequest{
					Object: runtime.RawExtension{
						Raw: mustMarshal(&apps.App{
							ObjectMeta: metav1.ObjectMeta{
								Name: "myapp-invalid-name",
								Labels: map[string]string{
									labelAppName: "myapp",
								},
							},
						}),
					},
				},
			},
			wantAllow: false,
		},
	}

	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			resp := v.Handle(t.Context(), tc.req)
			if resp.Allowed != tc.wantAllow {
				t.Errorf("Handle() allowed = %v, want %v. Message: %s", resp.Allowed, tc.wantAllow, resp.Result.Message)
			}
			if tc.wantStatus != 0 && resp.Result.Code != tc.wantStatus {
				t.Errorf("Handle() status code = %d, want %d", resp.Result.Code, tc.wantStatus)
			}
		})
	}
}

func TestAppRolloutValidator_Handle(t *testing.T) {
	scheme := runtime.NewScheme()
	apps.AddToScheme(scheme)

	decoder := serializer.NewCodecFactory(scheme).UniversalDeserializer()
	v := &appRolloutValidator{decoder: decoder}

	tests := []struct {
		name      string
		req       admission.Request
		wantAllow bool
	}{
		{
			name: "DecodeError",
			req: admission.Request{
				AdmissionRequest: admissionv1.AdmissionRequest{
					Object: runtime.RawExtension{Raw: []byte("invalid json")},
				},
			},
			wantAllow: false,
		},
		{
			name: "Allowed",
			req: admission.Request{
				AdmissionRequest: admissionv1.AdmissionRequest{
					Object: runtime.RawExtension{
						Raw: mustMarshal(&apps.AppRollout{
							ObjectMeta: metav1.ObjectMeta{
								Name: "my-rollout",
							},
							Spec: apps.AppRolloutSpec{
								AppName: "my-app",
							},
						}),
					},
				},
			},
			wantAllow: true,
		},
		{
			name: "Denied_ValidationFailure",
			req: admission.Request{
				AdmissionRequest: admissionv1.AdmissionRequest{
					Object: runtime.RawExtension{
						Raw: mustMarshal(&apps.AppRollout{
							ObjectMeta: metav1.ObjectMeta{
								Name: "my-rollout",
							},
							Spec: apps.AppRolloutSpec{
								AppName: "", // Missing app name
							},
						}),
					},
				},
			},
			wantAllow: false,
		},
	}

	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			resp := v.Handle(t.Context(), tc.req)
			if resp.Allowed != tc.wantAllow {
				t.Errorf("Handle() allowed = %v, want %v. Message: %s", resp.Allowed, tc.wantAllow, resp.Result.Message)
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
