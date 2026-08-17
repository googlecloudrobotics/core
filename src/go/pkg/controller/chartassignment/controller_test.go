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

package chartassignment

import (
	"context"
	"strings"
	"testing"

	"github.com/golang/mock/gomock"
	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"github.com/googlecloudrobotics/core/src/go/pkg/gcr"
	corev1 "k8s.io/api/core/v1"
	k8serrors "k8s.io/apimachinery/pkg/api/errors"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/client-go/tools/record"
	"k8s.io/client-go/util/workqueue"
	kclient "sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/client/fake"
	"sigs.k8s.io/controller-runtime/pkg/reconcile"
)

func TestReconciler_ensureNamespace(t *testing.T) {
	client := fake.NewClientBuilder().WithScheme(newTestScheme()).Build()

	r := &Reconciler{
		kube: client,
	}

	as := &apps.ChartAssignment{
		ObjectMeta: metav1.ObjectMeta{
			Name: "test-app",
			UID:  "test-uid",
		},
		Spec: apps.ChartAssignmentSpec{
			NamespaceName: "app-ns",
		},
	}

	ctx := t.Context()
	ns, err := r.ensureNamespace(ctx, as)
	if err != nil {
		t.Fatalf("ensureNamespace failed: %v", err)
	}

	if ns.Name != "app-ns" {
		t.Errorf("Expected namespace name 'app-ns', got %q", ns.Name)
	}

	// Verify namespace was created in fake client
	var createdNS corev1.Namespace
	err = client.Get(ctx, kclient.ObjectKey{Name: "app-ns"}, &createdNS)
	if err != nil {
		t.Fatalf("Namespace not found in fake client: %v", err)
	}
	if createdNS.Labels["app"] != "test-app" {
		t.Errorf("Expected label app=test-app, got %v", createdNS.Labels["app"])
	}
}

func TestReconciler_ensureSecrets(t *testing.T) {
	secret := &corev1.Secret{
		ObjectMeta: metav1.ObjectMeta{
			Name:      "test-secret",
			Namespace: "default",
			Labels: map[string]string{
				"cloudrobotics.com/copy-to-chart-namespaces": "true",
			},
		},
		Data: map[string][]byte{"foo": []byte("bar")},
	}

	client := fake.NewClientBuilder().WithScheme(newTestScheme()).WithObjects(secret).Build()

	r := &Reconciler{
		kube: client,
	}

	as := &apps.ChartAssignment{
		Spec: apps.ChartAssignmentSpec{
			NamespaceName: "app-ns",
		},
	}

	err := r.ensureSecrets(t.Context(), as)
	if err != nil {
		t.Fatalf("ensureSecrets failed: %v", err)
	}

	var copiedSecret corev1.Secret
	err = client.Get(t.Context(), kclient.ObjectKey{Namespace: "app-ns", Name: "test-secret"}, &copiedSecret)
	if err != nil {
		t.Fatalf("Secret not copied to app-ns: %v", err)
	}
}

func TestReconciler_Reconcile(t *testing.T) {
	scheme := runtime.NewScheme()
	apps.AddToScheme(scheme)
	corev1.AddToScheme(scheme)

	asName := "test-app"
	nsName := "app-ns"
	now := metav1.Now()

	tests := []struct {
		name         string
		reqName      string
		as           *apps.ChartAssignment
		objects      []kclient.Object
		cloud        bool
		mockReleases func(ctx context.Context, ctrl *gomock.Controller) *releases
		wantResult   reconcile.Result
		wantErr      bool
		verify       func(t *testing.T, client kclient.Client)
	}{
		{
			name:       "NotFound",
			reqName:    "non-existent",
			wantResult: reconcile.Result{},
		},
		{
			name:    "CloudRobotAssignment",
			reqName: asName,
			cloud:   true,
			as: &apps.ChartAssignment{
				ObjectMeta: metav1.ObjectMeta{
					Name: asName,
					Labels: map[string]string{
						"cloudrobotics.com/robot-name": "my-robot",
					},
				},
				Spec: apps.ChartAssignmentSpec{
					NamespaceName: nsName,
				},
			},
			wantResult: reconcile.Result{},
			verify: func(t *testing.T, client kclient.Client) {
				var ns corev1.Namespace
				err := client.Get(t.Context(), kclient.ObjectKey{Name: nsName}, &ns)
				if !k8serrors.IsNotFound(err) {
					t.Errorf("Expected namespace to not exist, got: %v", err)
				}
			},
		},
		{
			name:    "NamespaceDeletion",
			reqName: asName,
			as: &apps.ChartAssignment{
				ObjectMeta: metav1.ObjectMeta{
					Name:       asName,
					Generation: 1,
				},
				Spec: apps.ChartAssignmentSpec{
					NamespaceName: nsName,
				},
			},
			objects: []kclient.Object{
				&corev1.Namespace{
					ObjectMeta: metav1.ObjectMeta{
						Name:              nsName,
						DeletionTimestamp: &now,
						Finalizers:        []string{"kubernetes"},
					},
				},
			},
			wantResult: reconcile.Result{Requeue: true, RequeueAfter: requeueFast},
		},
		{
			name:    "MissingServiceAccount",
			reqName: asName,
			as: &apps.ChartAssignment{
				ObjectMeta: metav1.ObjectMeta{
					Name:       asName,
					Generation: 1,
				},
				Spec: apps.ChartAssignmentSpec{
					NamespaceName: nsName,
					Chart: apps.AssignedChart{
						Inline: "abc",
					},
				},
			},
			objects: []kclient.Object{
				&corev1.Namespace{
					ObjectMeta: metav1.ObjectMeta{
						Name:              nsName,
						CreationTimestamp: metav1.Now(),
					},
				},
				&corev1.Secret{
					ObjectMeta: metav1.ObjectMeta{
						Name:      gcr.SecretName,
						Namespace: nsName,
					},
				},
			},
			wantResult: reconcile.Result{Requeue: true, RequeueAfter: requeueFast},
		},
		{
			name:    "Delete",
			reqName: asName,
			as: &apps.ChartAssignment{
				ObjectMeta: metav1.ObjectMeta{
					Name:              asName,
					DeletionTimestamp: &now,
					Finalizers:        []string{finalizer},
				},
				Spec: apps.ChartAssignmentSpec{
					NamespaceName: nsName,
				},
			},
			mockReleases: func(ctx context.Context, ctrl *gomock.Controller) *releases {
				mockSynk := NewMockInterface(ctrl)
				mockSynk.EXPECT().Delete(gomock.Any(), asName).Return(nil).AnyTimes()
				return &releases{
					ctx: ctx,
					m: map[string]*release{
						asName: {
							ctx:    ctx,
							status: releaseStatus{phase: apps.ChartAssignmentPhaseDeleted},
						},
					},
					synk: mockSynk,
				}
			},
			wantResult: reconcile.Result{Requeue: true, RequeueAfter: requeueFast},
			verify: func(t *testing.T, client kclient.Client) {
				var updatedAs apps.ChartAssignment
				err := client.Get(t.Context(), kclient.ObjectKey{Name: asName}, &updatedAs)
				if !k8serrors.IsNotFound(err) {
					t.Errorf("Expected ChartAssignment to be deleted, but found: %v", updatedAs)
				}
			},
		},
		{
			name:    "Success",
			reqName: asName,
			as: &apps.ChartAssignment{
				ObjectMeta: metav1.ObjectMeta{
					Name:       asName,
					Generation: 1,
				},
				Spec: apps.ChartAssignmentSpec{
					NamespaceName: nsName,
					Chart: apps.AssignedChart{
						Inline: "abc",
					},
				},
			},
			mockReleases: func(ctx context.Context, ctrl *gomock.Controller) *releases {
				return &releases{
					ctx: ctx,
					m: map[string]*release{
						asName: {
							ctx:    ctx,
							status: releaseStatus{phase: apps.ChartAssignmentPhaseSettled},
							gen:    1,
						},
					},
				}
			},
			wantResult: reconcile.Result{Requeue: true, RequeueAfter: requeueSlow},
			verify: func(t *testing.T, client kclient.Client) {
				var updatedAs apps.ChartAssignment
				err := client.Get(t.Context(), kclient.ObjectKey{Name: asName}, &updatedAs)
				if err != nil {
					t.Fatalf("Failed to get ChartAssignment: %v", err)
				}
				if updatedAs.Status.Phase != apps.ChartAssignmentPhaseReady {
					t.Errorf("Expected phase Ready, got %q", updatedAs.Status.Phase)
				}
				if len(updatedAs.Finalizers) != 1 || updatedAs.Finalizers[0] != finalizer {
					t.Errorf("Expected finalizer %q, got %v", finalizer, updatedAs.Finalizers)
				}
			},
		},
		{
			name:    "PodsNotReady",
			reqName: asName,
			as: &apps.ChartAssignment{
				ObjectMeta: metav1.ObjectMeta{
					Name:       asName,
					Generation: 1,
				},
				Spec: apps.ChartAssignmentSpec{
					NamespaceName: nsName,
					Chart: apps.AssignedChart{
						Inline: "abc",
					},
				},
			},
			objects: []kclient.Object{
				&corev1.Namespace{
					ObjectMeta: metav1.ObjectMeta{
						Name: nsName,
					},
				},
				&corev1.Pod{
					ObjectMeta: metav1.ObjectMeta{
						Name:      "pod-1",
						Namespace: nsName,
					},
					Status: corev1.PodStatus{
						Phase: corev1.PodRunning,
					},
				},
				&corev1.Pod{
					ObjectMeta: metav1.ObjectMeta{
						Name:      "pod-2",
						Namespace: nsName,
					},
					Status: corev1.PodStatus{
						Phase: corev1.PodPending,
					},
				},
			},
			mockReleases: func(ctx context.Context, ctrl *gomock.Controller) *releases {
				return &releases{
					ctx: ctx,
					m: map[string]*release{
						asName: {
							ctx:    ctx,
							status: releaseStatus{phase: apps.ChartAssignmentPhaseSettled},
							gen:    1,
						},
					},
				}
			},
			wantResult: reconcile.Result{Requeue: true, RequeueAfter: requeueFast},
			verify: func(t *testing.T, client kclient.Client) {
				var updatedAs apps.ChartAssignment
				err := client.Get(t.Context(), kclient.ObjectKey{Name: asName}, &updatedAs)
				if err != nil {
					t.Fatalf("Failed to get ChartAssignment: %v", err)
				}
				if updatedAs.Status.Phase != apps.ChartAssignmentPhaseSettled {
					t.Errorf("Expected phase Settled, got %q", updatedAs.Status.Phase)
				}
				var readyCond *apps.ChartAssignmentCondition
				for _, c := range updatedAs.Status.Conditions {
					if c.Type == apps.ChartAssignmentConditionReady {
						readyCond = &c
						break
					}
				}
				if readyCond == nil {
					t.Fatal("Ready condition not found")
				}
				if readyCond.Status != corev1.ConditionFalse {
					t.Errorf("Expected Ready condition status False, got %v", readyCond.Status)
				}
				if !strings.Contains(readyCond.Message, "1/2 pods are running") {
					t.Errorf("Unexpected condition message: %q", readyCond.Message)
				}
			},
		},
	}

	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			ctrl := gomock.NewController(t)
			defer ctrl.Finish()

			var initObjs []kclient.Object
			if tc.as != nil {
				initObjs = append(initObjs, tc.as)
			}
			initObjs = append(initObjs, tc.objects...)

			client := fake.NewClientBuilder().
				WithScheme(scheme).
				WithStatusSubresource(&apps.ChartAssignment{}).
				WithObjects(initObjs...).
				Build()

			var rels *releases
			if tc.mockReleases != nil {
				rels = tc.mockReleases(t.Context(), ctrl)
			} else {
				rels = &releases{
					ctx: t.Context(),
					m:   map[string]*release{},
				}
			}

			r := &Reconciler{
				kube:     client,
				releases: rels,
				cloud:    tc.cloud,
				recorder: &record.FakeRecorder{},
			}

			req := reconcile.Request{
				NamespacedName: kclient.ObjectKey{Name: tc.reqName},
			}

			res, err := r.Reconcile(t.Context(), req)
			if (err != nil) != tc.wantErr {
				t.Fatalf("Reconcile() error = %v, wantErr %v", err, tc.wantErr)
			}
			if res != tc.wantResult {
				t.Errorf("Reconcile() res = %v, want %v", res, tc.wantResult)
			}

			if tc.verify != nil {
				tc.verify(t, client)
			}
		})
	}
}

func newTestScheme() *runtime.Scheme {
	scheme := runtime.NewScheme()
	apps.AddToScheme(scheme)
	corev1.AddToScheme(scheme)
	return scheme
}

func TestReconciler_ensureServiceAccount_Success(t *testing.T) {
	ns := &corev1.Namespace{
		ObjectMeta: metav1.ObjectMeta{
			Name: "app-ns",
		},
	}

	as := &apps.ChartAssignment{
		Spec: apps.ChartAssignmentSpec{
			NamespaceName: "app-ns",
		},
	}

	secret := &corev1.Secret{
		ObjectMeta: metav1.ObjectMeta{
			Name:      gcr.SecretName,
			Namespace: "app-ns",
		},
	}

	sa := &corev1.ServiceAccount{
		ObjectMeta: metav1.ObjectMeta{
			Name:      "default",
			Namespace: "app-ns",
		},
	}

	client := fake.NewClientBuilder().
		WithScheme(newTestScheme()).
		WithObjects(ns, as, secret, sa).
		Build()

	r := &Reconciler{
		kube: client,
	}

	err := r.ensureServiceAccount(t.Context(), ns, as)
	if err != nil {
		t.Fatalf("ensureServiceAccount failed: %v", err)
	}

	// Verify SA was updated with image pull secret
	var updatedSA corev1.ServiceAccount
	err = client.Get(t.Context(), kclient.ObjectKey{Namespace: "app-ns", Name: "default"}, &updatedSA)
	if err != nil {
		t.Fatalf("Failed to get SA: %v", err)
	}

	found := false
	for _, s := range updatedSA.ImagePullSecrets {
		if s.Name == gcr.SecretName {
			found = true
			break
		}
	}
	if !found {
		t.Errorf("gcr-json-key secret not found in SA image pull secrets: %v", updatedSA.ImagePullSecrets)
	}
}

func TestReconciler_enqueueForPod(t *testing.T) {
	scheme := runtime.NewScheme()
	apps.AddToScheme(scheme)
	corev1.AddToScheme(scheme)

	ca := &apps.ChartAssignment{
		ObjectMeta: metav1.ObjectMeta{
			Name: "my-ca",
		},
		Spec: apps.ChartAssignmentSpec{
			NamespaceName: "my-ns",
		},
	}

	client := fake.NewClientBuilder().
		WithScheme(scheme).
		WithIndex(&apps.ChartAssignment{}, fieldIndexNamespace, func(o kclient.Object) []string {
			return []string{o.(*apps.ChartAssignment).Spec.NamespaceName}
		}).
		WithObjects(ca).
		Build()

	r := &Reconciler{
		kube: client,
	}

	q := workqueue.NewTypedRateLimitingQueue(workqueue.DefaultTypedControllerRateLimiter[reconcile.Request]())
	pod := &corev1.Pod{
		ObjectMeta: metav1.ObjectMeta{
			Name:      "my-pod",
			Namespace: "my-ns",
		},
	}

	r.enqueueForPod(t.Context(), pod, q)

	if q.Len() != 1 {
		t.Errorf("Expected 1 item in queue, got %d", q.Len())
	}
	req, _ := q.Get()
	if req.Name != "my-ca" {
		t.Errorf("Expected enqueued CA 'my-ca', got %q", req.Name)
	}
}
