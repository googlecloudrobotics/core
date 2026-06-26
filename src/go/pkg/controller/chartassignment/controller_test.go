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
	"testing"

	"github.com/golang/mock/gomock"
	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	k8serrors "k8s.io/apimachinery/pkg/api/errors"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/client-go/tools/record"
	kclient "sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/client/fake"
	"sigs.k8s.io/controller-runtime/pkg/reconcile"
)

func TestReconciler_Reconcile_NotFound(t *testing.T) {
	client := fake.NewClientBuilder().WithScheme(newTestScheme()).Build()

	r := &Reconciler{
		kube: client,
	}

	req := reconcile.Request{
		NamespacedName: kclient.ObjectKey{Name: "non-existent"},
	}

	// When a resource is not found, the reconciler should return success (nil error)
	// and not requeue, as there's nothing left to reconcile.
	res, err := r.Reconcile(t.Context(), req)
	if err != nil {
		t.Fatalf("Reconcile failed: %v", err)
	}
	if res.Requeue || res.RequeueAfter != 0 {
		t.Errorf("Unexpected result: %v", res)
	}
}

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

func TestReconciler_Reconcile_Delete(t *testing.T) {
	ctrl := gomock.NewController(t)
	defer ctrl.Finish()

	now := metav1.Now()
	as := &apps.ChartAssignment{
		ObjectMeta: metav1.ObjectMeta{
			Name:              "test-app",
			DeletionTimestamp: &now,
			Finalizers:        []string{finalizer},
		},
		Spec: apps.ChartAssignmentSpec{
			NamespaceName: "app-ns",
		},
	}

	client := fake.NewClientBuilder().WithScheme(newTestScheme()).WithObjects(as).Build()
	mockSynk := NewMockInterface(ctrl)

	releases := &releases{
		ctx:  t.Context(),
		m:    map[string]*release{},
		synk: mockSynk,
	}

	r := &Reconciler{
		kube:     client,
		releases: releases,
		recorder: &record.FakeRecorder{},
	}

	// Mock release status as Deleted
	releases.m["test-app"] = &release{
		ctx:    t.Context(),
		status: releaseStatus{phase: apps.ChartAssignmentPhaseDeleted},
	}

	mockSynk.EXPECT().Delete(gomock.Any(), "test-app").Return(nil).AnyTimes()

	req := reconcile.Request{
		NamespacedName: kclient.ObjectKey{Name: "test-app"},
	}

	res, err := r.Reconcile(t.Context(), req)
	if err != nil {
		t.Fatalf("Reconcile failed: %v", err)
	}

	if !res.Requeue {
		t.Errorf("Expected requeue during deletion")
	}

	// Verify ChartAssignment was deleted (no more finalizers)
	var updatedAs apps.ChartAssignment
	err = client.Get(t.Context(), kclient.ObjectKey{Name: "test-app"}, &updatedAs)
	if !k8serrors.IsNotFound(err) {
		t.Errorf("Expected ChartAssignment to be deleted, but found: %v", updatedAs)
	}
}

func newTestScheme() *runtime.Scheme {
	scheme := runtime.NewScheme()
	apps.AddToScheme(scheme)
	corev1.AddToScheme(scheme)
	return scheme
}
