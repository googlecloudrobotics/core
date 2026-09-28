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

package main

import (
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"sync/atomic"
	"testing"
	"testing/synctest"
	"time"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/client-go/rest"
)

func TestCheckClusterHealth(t *testing.T) {
	t.Run("eventually_healthy_after_sleep", func(t *testing.T) {
		synctest.Test(t, func(t *testing.T) {
			var calls atomic.Int32
			srv := httptest.NewTestServer(t, http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
				n := calls.Add(1)
				phase := corev1.PodPending
				if n >= 3 {
					phase = corev1.PodRunning
				}
				podList := &corev1.PodList{
					TypeMeta: metav1.TypeMeta{Kind: "PodList", APIVersion: "v1"},
					Items: []corev1.Pod{
						{
							ObjectMeta: metav1.ObjectMeta{Name: "test-pod", Namespace: "default"},
							Status: corev1.PodStatus{
								Phase: phase,
								ContainerStatuses: []corev1.ContainerStatus{
									{
										Name:  "app",
										State: corev1.ContainerState{Running: &corev1.ContainerStateRunning{}},
									},
								},
							},
						},
					},
				}
				w.Header().Set("Content-Type", "application/json")
				_ = json.NewEncoder(w).Encode(podList)
			}))

			cfg := &rest.Config{
				Host:      srv.URL,
				Transport: srv.Client().Transport,
			}
			start := time.Now()
			if err := checkClusterHealth(t.Context(), "test-cluster", cfg); err != nil {
				t.Fatalf("checkClusterHealth failed: %v", err)
			}
			if elapsed := time.Since(start); elapsed != 20*time.Second {
				t.Errorf("expected simulated elapsed time of 20s, got %v", elapsed)
			}
		})
	})

	t.Run("times_out_after_5_minutes", func(t *testing.T) {
		synctest.Test(t, func(t *testing.T) {
			srv := httptest.NewTestServer(t, http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
				podList := &corev1.PodList{
					TypeMeta: metav1.TypeMeta{Kind: "PodList", APIVersion: "v1"},
					Items: []corev1.Pod{
						{
							ObjectMeta: metav1.ObjectMeta{Name: "stuck-pod", Namespace: "default"},
							Status:     corev1.PodStatus{Phase: corev1.PodPending},
						},
					},
				}
				w.Header().Set("Content-Type", "application/json")
				_ = json.NewEncoder(w).Encode(podList)
			}))

			cfg := &rest.Config{
				Host:      srv.URL,
				Transport: srv.Client().Transport,
			}
			start := time.Now()
			err := checkClusterHealth(t.Context(), "test-cluster", cfg)
			if err == nil {
				t.Fatal("expected timeout error, got nil")
			}
			if elapsed := time.Since(start); elapsed != podInitializationTimeout {
				t.Errorf("expected simulated elapsed time of %v, got %v", podInitializationTimeout, elapsed)
			}
		})
	})
}

func TestCheckAppRollouts(t *testing.T) {
	t.Run("eventually_settled_after_error_and_retry", func(t *testing.T) {
		synctest.Test(t, func(t *testing.T) {
			var listCalls atomic.Int32
			srv := httptest.NewTestServer(t, http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
				w.Header().Set("Content-Type", "application/json")
				switch r.URL.Path {
				case "/api":
					_, _ = w.Write([]byte(`{"kind":"APIVersions","versions":["v1"]}`))
				case "/apis":
					_, _ = w.Write([]byte(`{"kind":"APIGroupList","apiVersion":"v1","groups":[{"name":"apps.cloudrobotics.com","versions":[{"groupVersion":"apps.cloudrobotics.com/v1alpha1","version":"v1alpha1"}],"preferredVersion":{"groupVersion":"apps.cloudrobotics.com/v1alpha1","version":"v1alpha1"}}]}`))
				case "/apis/apps.cloudrobotics.com/v1alpha1":
					_, _ = w.Write([]byte(`{"kind":"APIResourceList","apiVersion":"v1","groupVersion":"apps.cloudrobotics.com/v1alpha1","resources":[{"name":"approllouts","singularName":"approllout","namespaced":false,"kind":"AppRollout","verbs":["get","list"]}]}`))
				case "/apis/apps.cloudrobotics.com/v1alpha1/approllouts":
					n := listCalls.Add(1)
					if n == 1 {
						w.WriteHeader(http.StatusInternalServerError)
						return
					}
					condStatus := corev1.ConditionFalse
					if n >= 3 {
						condStatus = corev1.ConditionTrue
					}
					rollouts := &apps.AppRolloutList{
						TypeMeta: metav1.TypeMeta{Kind: "AppRolloutList", APIVersion: "apps.cloudrobotics.com/v1alpha1"},
						Items: []apps.AppRollout{
							{
								TypeMeta:   metav1.TypeMeta{Kind: "AppRollout", APIVersion: "apps.cloudrobotics.com/v1alpha1"},
								ObjectMeta: metav1.ObjectMeta{Name: "test-app"},
								Status: apps.AppRolloutStatus{
									Conditions: []apps.AppRolloutCondition{
										{Type: apps.AppRolloutConditionSettled, Status: condStatus},
									},
								},
							},
						},
					}
					_ = json.NewEncoder(w).Encode(rollouts)
				default:
					w.WriteHeader(http.StatusNotFound)
				}
			}))

			cfg := &rest.Config{
				Host:      srv.URL,
				Transport: srv.Client().Transport,
			}
			start := time.Now()
			if err := checkAppRollouts(t.Context(), "test-cloud", cfg); err != nil {
				t.Fatalf("checkAppRollouts failed: %v", err)
			}
			// Two 15s poll intervals (first on list error, second on unset condition) = 30s
			if elapsed := time.Since(start); elapsed != 30*time.Second {
				t.Errorf("expected simulated elapsed time of 30s, got %v", elapsed)
			}
		})
	})

	t.Run("times_out_after_7_minutes", func(t *testing.T) {
		synctest.Test(t, func(t *testing.T) {
			srv := httptest.NewTestServer(t, http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
				w.Header().Set("Content-Type", "application/json")
				switch r.URL.Path {
				case "/api":
					_, _ = w.Write([]byte(`{"kind":"APIVersions","versions":["v1"]}`))
				case "/apis":
					_, _ = w.Write([]byte(`{"kind":"APIGroupList","apiVersion":"v1","groups":[{"name":"apps.cloudrobotics.com","versions":[{"groupVersion":"apps.cloudrobotics.com/v1alpha1","version":"v1alpha1"}],"preferredVersion":{"groupVersion":"apps.cloudrobotics.com/v1alpha1","version":"v1alpha1"}}]}`))
				case "/apis/apps.cloudrobotics.com/v1alpha1":
					_, _ = w.Write([]byte(`{"kind":"APIResourceList","apiVersion":"v1","groupVersion":"apps.cloudrobotics.com/v1alpha1","resources":[{"name":"approllouts","singularName":"approllout","namespaced":false,"kind":"AppRollout","verbs":["get","list"]}]}`))
				case "/apis/apps.cloudrobotics.com/v1alpha1/approllouts":
					rollouts := &apps.AppRolloutList{
						TypeMeta: metav1.TypeMeta{Kind: "AppRolloutList", APIVersion: "apps.cloudrobotics.com/v1alpha1"},
						Items: []apps.AppRollout{
							{
								TypeMeta:   metav1.TypeMeta{Kind: "AppRollout", APIVersion: "apps.cloudrobotics.com/v1alpha1"},
								ObjectMeta: metav1.ObjectMeta{Name: "unhealthy-app"},
								Status: apps.AppRolloutStatus{
									Conditions: []apps.AppRolloutCondition{
										{Type: apps.AppRolloutConditionSettled, Status: corev1.ConditionFalse},
									},
								},
							},
						},
					}
					_ = json.NewEncoder(w).Encode(rollouts)
				default:
					w.WriteHeader(http.StatusNotFound)
				}
			}))

			cfg := &rest.Config{
				Host:      srv.URL,
				Transport: srv.Client().Transport,
			}
			start := time.Now()
			err := checkAppRollouts(t.Context(), "test-cloud", cfg)
			if err == nil {
				t.Fatal("expected timeout error, got nil")
			}
			if elapsed := time.Since(start); elapsed != appInitializationTimeout {
				t.Errorf("expected simulated elapsed time of %v, got %v", appInitializationTimeout, elapsed)
			}
		})
	})
}
