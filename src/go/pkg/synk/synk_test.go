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

package synk

import (
	"encoding/json"
	"errors"
	"fmt"
	"reflect"
	"strings"
	"testing"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	apiextensions "k8s.io/apiextensions-apiserver/pkg/apis/apiextensions/v1"
	k8serrors "k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/api/meta"
	"k8s.io/apimachinery/pkg/api/meta/testrestmapper"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/apimachinery/pkg/util/strategicpatch"
	"k8s.io/client-go/discovery"
	dynamicfake "k8s.io/client-go/dynamic/fake"
	"k8s.io/client-go/kubernetes/scheme"
	k8stest "k8s.io/client-go/testing"
	"sigs.k8s.io/yaml"
)

// The fake discovery client has quite a bit of code but still returns empty results
// even when pointed at the Fake object of the dynamic client.
// Thus we implement our own static one.
type fakeCachedDiscoveryClient struct {
	discovery.CachedDiscoveryInterface
	resources map[string]*metav1.APIResourceList
}

func (d *fakeCachedDiscoveryClient) Invalidate() {}

func (d *fakeCachedDiscoveryClient) ServerGroupsAndResources() ([]*metav1.APIGroup, []*metav1.APIResourceList, error) {
	var list []*metav1.APIResourceList
	for _, l := range d.resources {
		list = append(list, l)
	}
	return nil, list, nil
}

func (d *fakeCachedDiscoveryClient) ServerResourcesForGroupVersion(gv string) (*metav1.APIResourceList, error) {
	if list, ok := d.resources[gv]; ok {
		return list, nil
	}
	return nil, k8serrors.NewNotFound(schema.GroupResource{}, gv)
}

type fixture struct {
	*testing.T
	fake      *k8stest.Fake
	discovery *fakeCachedDiscoveryClient

	// Starting state the respective client will report.
	objects []runtime.Object
	// Actions we want to see called against the respective client.
	actions []k8stest.Action
}

func newFixture(t *testing.T) *fixture {
	t.Helper()
	f := &fixture{
		T:         t,
		discovery: &fakeCachedDiscoveryClient{resources: make(map[string]*metav1.APIResourceList)},
	}
	// Add default v1 resources.
	f.discovery.resources["v1"] = &metav1.APIResourceList{
		GroupVersion: "v1",
		APIResources: []metav1.APIResource{
			{Kind: "Pod", Namespaced: true},
			{Kind: "Namespace", Namespaced: false},
		},
	}
	return f
}

func (f *fixture) newSynk() *Synk {
	sc := runtime.NewScheme()
	scheme.AddToScheme(sc)
	apps.AddToScheme(sc)          // For tests with CRDs.
	apiextensions.AddToScheme(sc) // For CRD tests.
	var (
		client = dynamicfake.NewSimpleDynamicClient(sc, f.objects...)
		s      = New(client, f.discovery)
	)
	s.mapper = testrestmapper.TestOnlyStaticRESTMapper(sc)
	s.resetMapper = func() {}
	s.workers = 1
	f.fake = &client.Fake
	return s
}

func (f *fixture) addObjects(objs ...runtime.Object) {
	f.objects = append(f.objects, objs...)
}

func (f *fixture) expectActions(as ...k8stest.Action) {
	f.actions = append(f.actions, as...)
}

func (f *fixture) verifyWriteActions() {
	writes := filterReadActions(f.fake.Actions())

	if !reflect.DeepEqual(writes, f.actions) {
		f.Errorf("writes did not match")
		f.Logf("received:")
		for i, a := range writes {
			f.Logf("%d: %s", i, sprintAction(a))
		}
		f.Logf("expected:")
		for i, a := range f.actions {
			f.Logf("%d: %s", i, sprintAction(a))
		}
	}
}

func TestSynk_IsTransientErr(t *testing.T) {
	tests := []struct {
		name string
		errs []error
		want bool
	}{
		{
			name: "transient",
			want: true,
			errs: []error{
				transientErr{errors.New("transientErr struct")},
				&transientErr{errors.New("transientErr pointer")},
				k8serrors.NewResourceExpired("gone"),
				k8serrors.NewForbidden(schema.GroupResource{
					Group:    "apps",
					Resource: "deployments",
				}, "my-deployment", errors.New("unable to create new content in namespace app-test-chart because it is being terminated")),
				k8serrors.NewConflict(schema.GroupResource{}, "", nil),
				k8serrors.NewAlreadyExists(schema.GroupResource{}, ""),
				k8serrors.NewNotFound(schema.GroupResource{}, ""),
				k8serrors.NewGone(""),
				k8serrors.NewInternalError(errors.New("internal")),
				k8serrors.NewServerTimeout(schema.GroupResource{}, "", 0),
				k8serrors.NewTimeoutError("", 0),
				k8serrors.NewTooManyRequests("", 0),
				k8serrors.NewServiceUnavailable(""),
				&discovery.ErrGroupDiscoveryFailed{Groups: map[schema.GroupVersion]error{}},
				&meta.NoKindMatchError{GroupKind: schema.GroupKind{Group: "gateway.envoyproxy.io", Kind: "SecurityPolicy"}},
				&meta.NoResourceMatchError{PartialResource: schema.GroupVersionResource{Group: "gateway.envoyproxy.io", Version: "v1alpha1", Resource: "securitypolicies"}},
			},
		},
		{
			name: "non-transient",
			want: false,
			errs: []error{
				errors.New("generic error"),
				k8serrors.NewUnauthorized("unauthorized"),
			},
		},
	}

	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			for _, err := range tc.errs {
				if got := IsTransientErr(err); got != tc.want {
					t.Errorf("IsTransientErr(%T) = %v, want %v", err, got, tc.want)
				}
			}
		})
	}
}

// TODO(rodrigoq): test Apply() directly rather than the private methods
func TestSynk_initialize(t *testing.T) {
	ctx := t.Context()
	s := newFixture(t).newSynk()

	_, _, err := s.initialize(ctx, &ApplyOptions{name: "test"},
		newUnstructured("v1", "Pod", "ns2", "pod1"),
		newUnstructured("apps/v1", "Deployment", "ns1", "deploy1"),
		newUnstructured("v1", "Pod", "ns1", "pod1"),
	)
	if err != nil {
		t.Fatal(err)
	}
	got, err := s.client.Resource(resourceSetGVR).Get(ctx, "test.v1", metav1.GetOptions{})
	if err != nil {
		t.Fatal(err)
	}
	var want unstructured.Unstructured
	unmarshalYAML(t, &want, `
apiVersion: apps.cloudrobotics.com/v1alpha1
kind: ResourceSet
metadata:
  labels:
    name: test
  name: test.v1
spec:
  resources:
  - version: v1
    kind: Pod
    items:
    - name: pod1
      namespace: ns1
    - name: pod1
      namespace: ns2
  - group: apps
    version: v1
    kind: Deployment
    items:
    - namespace: ns1
      name: deploy1
status:
  phase: Pending
`)
	if want.GetName() != got.GetName() {
		t.Errorf("expected name %q but got %q", want.GetName(), got.GetName())
	}
	wantPhase, _, _ := unstructured.NestedString(want.Object, "status", "phase")
	gotPhase, _, _ := unstructured.NestedString(got.Object, "status", "phase")
	if wantPhase != gotPhase {
		t.Errorf("expected status phase %q but got %q", wantPhase, gotPhase)
	}
	if !reflect.DeepEqual(want.Object["spec"], got.Object["spec"]) {
		t.Errorf("expected spec\n%v\nbut got\n%v", want.Object["spec"], got.Object["spec"])
	}
}

func TestSynk_updateResourceSetStatus(t *testing.T) {
	ctx := t.Context()
	f := newFixture(t)
	s := f.newSynk()

	rs := &apps.ResourceSet{
		ObjectMeta: metav1.ObjectMeta{Name: "set1"},
	}
	if err := s.createResourceSet(ctx, rs); err != nil {
		t.Fatal(err)
	}
	results := applyResults{
		"/v1/Pod/ns1/pod1": &applyResult{
			resource: newUnstructured("v1", "Pod", "ns1", "pod1"),
			action:   apps.ResourceActionCreate,
			err:      errors.New("oops"),
		},
		"/v1/Pod/ns1/pod2": &applyResult{
			resource: newUnstructured("v1", "Pod", "ns1", "pod2"),
			action:   apps.ResourceActionCreate,
		},
		"/v1/Pod/ns2/pod1": &applyResult{
			resource: newUnstructured("v1", "Pod", "ns2", "pod1"),
			action:   apps.ResourceActionUpdate,
		},
		"apps/v1/Deployment/ns1/deploy1": &applyResult{
			resource: newUnstructured("apps/v1", "Deployment", "ns1", "deploy1"),
			action:   apps.ResourceActionCreate,
		},
	}
	err := s.updateResourceSetStatus(ctx, rs, results)
	if err != nil {
		t.Fatal(err)
	}
	got, err := s.client.Resource(resourceSetGVR).Get(ctx, "set1", metav1.GetOptions{})
	if err != nil {
		t.Fatal(err)
	}
	var want unstructured.Unstructured
	unmarshalYAML(t, &want, `
apiVersion: apps.cloudrobotics.com/v1alpha1
kind: ResourceSet
metadata:
  name: set1
status:
  phase: Failed
  applied:
  failed:
  - version: v1
    kind: Pod
    items:
    - name: pod1
      namespace: ns1
      error: oops
      action: Create
  applied:
  - version: v1
    kind: Pod
    items:
    - name: pod2
      namespace: ns1
      action: Create
    - name: pod1
      namespace: ns2
      action: Update
  - group: apps
    version: v1
    kind: Deployment
    items:
    - namespace: ns1
      name: deploy1
      action: Create
`)
	if v, _, _ := unstructured.NestedString(got.Object, "status", "finishedAt"); v == "" {
		t.Errorf("finishedAt timestamp was not set")
	}
	// Remove unknown timestamps before running DeepEqual.
	unstructured.RemoveNestedField(got.Object, "status", "startedAt")
	unstructured.RemoveNestedField(got.Object, "status", "finishedAt")

	if !reflect.DeepEqual(got.Object["status"], want.Object["status"]) {
		t.Errorf("expected status:\n%q\nbut got:\n%q", want.Object["status"], got.Object["status"])
	}
}

// Hardcode some GVR mappings for easy use in tests. The only other way is
// setting up a full RestMapper.
var gvrs = map[string]schema.GroupVersionResource{
	"configmaps":  {Version: "v1", Resource: "configmaps"},
	"services":    {Version: "v1", Resource: "services"},
	"deployments": {Group: "apps", Version: "v1", Resource: "deployments"},
	"approllouts": {Group: "apps.cloudrobotics.com", Version: "v1alpha1", Resource: "approllouts"},
}

func TestSynk_applyAllIsUpdatingResources(t *testing.T) {
	// We have to use properly typed objects for strategic-merge-patch targets
	// as the patch operation will fail otherwise.
	var cmBefore, cmUpdate corev1.ConfigMap
	unmarshalYAML(t, &cmBefore, `
apiVersion: v1
kind: ConfigMap
metadata:
  namespace: foo1
  name: cm1
data:
  foo1: bar1
  foo2: bar2`)
	f := newFixture(t)
	// cm1 already exists beforehand, so we expect an update.
	f.addObjects(&cmBefore)

	unmarshalYAML(t, &cmUpdate, `
apiVersion: v1
kind: ConfigMap
metadata:
  namespace: foo1
  name: cm1
data:
  foo2: baz2
  foo3: bar3`)
	cm := toUnstructured(t, &cmUpdate)

	set := &apps.ResourceSet{}
	set.Name = "test.v1"
	set.UID = "deadbeef"

	// Note: We can't test applying an Unstructured object here, as the
	// fake client doesn't support strategic merge patches:
	// https://github.com/kubernetes/client-go/issues/613
	results, err := f.newSynk().applyAll(t.Context(), set, &ApplyOptions{name: "test"},
		cm.DeepCopy(),
	)
	if err != nil {
		t.Error(err)
		for _, res := range results {
			t.Log(res)
		}
		return
	}

	ownerRef := metav1.OwnerReference{
		APIVersion:         "apps.cloudrobotics.com/v1alpha1",
		Kind:               "ResourceSet",
		Name:               set.Name,
		UID:                set.UID,
		BlockOwnerDeletion: new(true),
	}
	cm.SetOwnerReferences([]metav1.OwnerReference{ownerRef})

	setAppliedAnnotation(cm)

	f.expectActions(
		k8stest.NewUpdateAction(gvrs["configmaps"], "foo1", cm),
	)
	f.verifyWriteActions()
}

func TestSynk_applyAllRejectsTerminatingResource(t *testing.T) {
	now := metav1.Now()
	var cmBefore corev1.ConfigMap
	unmarshalYAML(t, &cmBefore, `
apiVersion: v1
kind: ConfigMap
metadata:
  namespace: foo1
  name: cm1
data:
  foo1: bar1`)
	cmBefore.SetDeletionTimestamp(&now)
	cmBefore.SetFinalizers([]string{"example.com/test-finalizer"})

	f := newFixture(t)
	f.addObjects(&cmBefore)

	var cmUpdate corev1.ConfigMap
	unmarshalYAML(t, &cmUpdate, `
apiVersion: v1
kind: ConfigMap
metadata:
  namespace: foo1
  name: cm1
data:
  foo1: bar2`)
	cm := toUnstructured(t, &cmUpdate)

	set := &apps.ResourceSet{}
	set.Name = "test.v1"
	set.UID = "deadbeef"

	results, err := f.newSynk().applyAll(t.Context(), set, &ApplyOptions{name: "test"}, cm.DeepCopy())
	if err == nil {
		t.Fatal("expected error applying over terminating resource, got nil")
	}
	if !IsTransientErr(err) {
		t.Errorf("expected IsTransientErr(err) to be true, got false; err: %v", err)
	}
	if len(results) == 0 || results.list()[0].err == nil {
		t.Errorf("expected result to contain error, got results: %v", results)
	}
	if !strings.Contains(err.Error(), "is terminating") {
		t.Errorf("expected error message to contain 'is terminating', got: %v", err)
	}
	f.verifyWriteActions()
}

func TestSynk_applyAllReplacesHeadlessToHeadedService(t *testing.T) {
	var svcBefore, svcUpdate corev1.Service
	unmarshalYAML(t, &svcBefore, `
apiVersion: v1
kind: Service
metadata:
  namespace: foo1
  name: svc1
spec:
  clusterIP: None
  ports:
  - port: 8080`)
	svcBeforeUnstructured := toUnstructured(t, &svcBefore)
	setAppliedAnnotation(svcBeforeUnstructured)

	f := newFixture(t)
	f.addObjects(svcBeforeUnstructured)

	unmarshalYAML(t, &svcUpdate, `
apiVersion: v1
kind: Service
metadata:
  namespace: foo1
  name: svc1
spec:
  ports:
  - port: 8080`)
	svc := toUnstructured(t, &svcUpdate)

	set := &apps.ResourceSet{}
	set.Name = "test.v1"
	set.UID = "deadbeef"

	results, err := f.newSynk().applyAll(t.Context(), set, &ApplyOptions{name: "test"},
		svc.DeepCopy(),
	)
	if err != nil {
		t.Fatalf("applyAll() failed: %v", err)
	}

	res, ok := results["/v1/Service/foo1/svc1"]
	if !ok {
		t.Fatalf("result for svc1 not found")
	}
	if res.action != apps.ResourceActionReplace {
		t.Errorf("expected action %q, got %q", apps.ResourceActionReplace, res.action)
	}

	ownerRef := metav1.OwnerReference{
		APIVersion:         "apps.cloudrobotics.com/v1alpha1",
		Kind:               "ResourceSet",
		Name:               set.Name,
		UID:                set.UID,
		BlockOwnerDeletion: new(true),
	}
	svc.SetOwnerReferences([]metav1.OwnerReference{ownerRef})
	setAppliedAnnotation(svc)

	f.expectActions(
		k8stest.NewDeleteAction(gvrs["services"], "foo1", "svc1"),
		k8stest.NewCreateAction(gvrs["services"], "foo1", svc),
	)
	f.verifyWriteActions()
}

func TestSynk_applyAllSkipsIgnoredResources(t *testing.T) {
	var cmBefore corev1.ConfigMap
	unmarshalYAML(t, &cmBefore, `
apiVersion: v1
kind: ConfigMap
metadata:
  namespace: foo1
  name: cm1
  annotations:
    synk.cloudrobotics.com/ignore: "true"
data:
  foo1: bar1`)
	f := newFixture(t)
	f.addObjects(&cmBefore)

	// The updated resource we want to apply (from chart)
	var cmUpdate corev1.ConfigMap
	unmarshalYAML(t, &cmUpdate, `
apiVersion: v1
kind: ConfigMap
metadata:
  namespace: foo1
  name: cm1
data:
  foo1: baz1`) // Chart wants baz1, but we should ignore this and keep bar1
	cm := toUnstructured(t, &cmUpdate)

	set := &apps.ResourceSet{}
	set.Name = "test.v1"
	set.UID = "deadbeef"

	results, err := f.newSynk().applyAll(t.Context(), set, &ApplyOptions{name: "test"},
		cm.DeepCopy(),
	)
	if err != nil {
		t.Fatal(err)
	}

	// We expect an update action on the fake client.
	// The object in the update action should be `cmBefore` (with data: foo1: bar1)
	// but with owner references updated to point to `set`.
	expectedCm := toUnstructured(t, &cmBefore)
	ownerRef := metav1.OwnerReference{
		APIVersion:         "apps.cloudrobotics.com/v1alpha1",
		Kind:               "ResourceSet",
		Name:               set.Name,
		UID:                set.UID,
		BlockOwnerDeletion: new(true),
	}
	expectedCm.SetOwnerReferences([]metav1.OwnerReference{ownerRef})

	f.expectActions(
		k8stest.NewUpdateAction(gvrs["configmaps"], "foo1", expectedCm),
	)
	f.verifyWriteActions()

	// Verify that the result returned by applyAll also reflects the ignored state
	if len(results) != 1 {
		t.Fatalf("expected 1 result, got %d", len(results))
	}
	res, ok := results["/v1/ConfigMap/foo1/cm1"]
	if !ok {
		t.Fatalf("result for cm1 not found")
	}
	if res.err != nil {
		t.Errorf("expected no error, got %s", res.err)
	}
	if res.action != apps.ResourceActionIgnored {
		t.Errorf("expected action Ignored, got %s", res.action)
	}

	// Check that the returned resource has the old data
	val, _, _ := unstructured.NestedString(res.resource.Object, "data", "foo1")
	if val != "bar1" {
		t.Errorf("expected data.foo1 to be 'bar1' (ignored), but got %q", val)
	}
}

func TestSynk_applyAllIsCreatingResources(t *testing.T) {
	f := newFixture(t)

	// Support for a standard merge-patch was only added to the client-go testing
	// mock as of kubernetes-1.14.0, which we cannot upgrade to yet.
	// Thus we cannot valid the standard merge patch type for CRDs yet.
	rollout := newUnstructured("apps.cloudrobotics.com/v1alpha1", "AppRollout", "foo1", "rollout1")
	deploy := newUnstructured("apps/v1", "Deployment", "foo2", "dp1")

	set := &apps.ResourceSet{}
	set.Name = "test.v1"
	set.UID = "deadbeef"

	results, err := f.newSynk().applyAll(t.Context(), set, &ApplyOptions{name: "test"},
		rollout.DeepCopy(),
		deploy.DeepCopy(),
	)
	if err != nil {
		t.Error(err)
		for _, res := range results {
			t.Log(res)
		}
		return
	}

	ownerRef := metav1.OwnerReference{
		APIVersion:         "apps.cloudrobotics.com/v1alpha1",
		Kind:               "ResourceSet",
		Name:               set.Name,
		UID:                set.UID,
		BlockOwnerDeletion: new(true),
	}
	rollout.SetOwnerReferences([]metav1.OwnerReference{ownerRef})
	deploy.SetOwnerReferences([]metav1.OwnerReference{ownerRef})

	setAppliedAnnotation(deploy)
	setAppliedAnnotation(rollout)

	f.expectActions(
		k8stest.NewCreateAction(gvrs["approllouts"], "foo1", rollout),
		k8stest.NewCreateAction(gvrs["deployments"], "foo2", deploy),
	)
	f.verifyWriteActions()
}

func TestSynk_applyAllRetriesResourceExpired(t *testing.T) {
	// deploy is the input to applyAll(), annotatedDeploy is the expected output.
	// TODO(rodrigoq): change verifyWriteActions() to avoid this boilerplate
	deploy := newUnstructured("apps/v1", "Deployment", "foo1", "dp1")
	annotatedDeploy := deploy.DeepCopy()
	set := &apps.ResourceSet{}
	set.Name = "test.v1"
	set.UID = "deadbeef"
	ownerRef := metav1.OwnerReference{
		APIVersion:         "apps.cloudrobotics.com/v1alpha1",
		Kind:               "ResourceSet",
		Name:               set.Name,
		UID:                set.UID,
		BlockOwnerDeletion: new(true),
	}
	annotatedDeploy.SetOwnerReferences([]metav1.OwnerReference{ownerRef})
	setAppliedAnnotation(annotatedDeploy)
	emptyPatch := []byte(`{}`)

	tests := []struct {
		desc    string
		verb    string
		objects []runtime.Object
		actions []k8stest.Action
	}{{
		desc:    "create deployment returns ResourceExpired",
		verb:    "create",
		objects: []runtime.Object{},
		actions: []k8stest.Action{
			k8stest.NewCreateAction(gvrs["deployments"], "foo1", annotatedDeploy),
			k8stest.NewCreateAction(gvrs["deployments"], "foo1", annotatedDeploy),
		},
	}, {
		desc:    "patch deployment returns ResourceExpired",
		verb:    "patch",
		objects: []runtime.Object{annotatedDeploy},
		actions: []k8stest.Action{
			k8stest.NewPatchAction(gvrs["deployments"], "foo1", "dp1", types.StrategicMergePatchType, emptyPatch),
			k8stest.NewPatchAction(gvrs["deployments"], "foo1", "dp1", types.StrategicMergePatchType, emptyPatch),
		},
	}, {
		desc:    "update deployment returns ResourceExpired",
		verb:    "update",
		objects: []runtime.Object{deploy},
		actions: []k8stest.Action{
			k8stest.NewUpdateAction(gvrs["deployments"], "foo1", annotatedDeploy),
			k8stest.NewUpdateAction(gvrs["deployments"], "foo1", annotatedDeploy),
		},
	}}

	for _, tc := range tests {
		t.Run(tc.desc, func(t *testing.T) {
			f := newFixture(t)
			f.addObjects(tc.objects...)
			s := f.newSynk()
			// ResourceExpired should be treated as a transient error.
			f.fake.PrependReactor(tc.verb, "deployments", func(action k8stest.Action) (bool, runtime.Object, error) {
				return true, nil, k8serrors.NewResourceExpired("gone")
			})

			_, err := s.applyAll(t.Context(), set, &ApplyOptions{name: "test"},
				deploy.DeepCopy(),
			)
			if err == nil {
				t.Error("applyAll() succeeded unexpectedly, want ResourceExpired")
			}

			// applyAll() should retry once before failing.
			f.expectActions(tc.actions...)
			f.verifyWriteActions()
		})
	}
}

func TestSynk_applyAllPrefetchesMultipleResources(t *testing.T) {
	set := &apps.ResourceSet{}
	set.Name = "test.v1"
	set.UID = "deadbeef"
	ownerRef := metav1.OwnerReference{
		APIVersion:         "apps.cloudrobotics.com/v1alpha1",
		Kind:               "ResourceSet",
		Name:               set.Name,
		UID:                set.UID,
		BlockOwnerDeletion: new(true),
	}

	t.Run("lists group with >=2 resources and gets single resource", func(t *testing.T) {
		var cm1Before corev1.ConfigMap
		unmarshalYAML(t, &cm1Before, `
apiVersion: v1
kind: ConfigMap
metadata:
  namespace: foo1
  name: cm1
data:
  foo1: bar1`)
		f := newFixture(t)
		f.addObjects(&cm1Before)

		var cm1Update, cm2Create corev1.ConfigMap
		unmarshalYAML(t, &cm1Update, `
apiVersion: v1
kind: ConfigMap
metadata:
  namespace: foo1
  name: cm1
data:
  foo1: baz1`)
		unmarshalYAML(t, &cm2Create, `
apiVersion: v1
kind: ConfigMap
metadata:
  namespace: foo1
  name: cm2
data:
  foo2: bar2`)
		cm1 := toUnstructured(t, &cm1Update)
		cm2 := toUnstructured(t, &cm2Create)
		deploy := newUnstructured("apps/v1", "Deployment", "foo1", "dp1")

		s := f.newSynk()
		_, err := s.applyAll(t.Context(), set, &ApplyOptions{name: "test"},
			cm1.DeepCopy(),
			cm2.DeepCopy(),
			deploy.DeepCopy(),
		)
		if err != nil {
			t.Fatalf("applyAll() failed: %v", err)
		}

		var cmLists, cmGets, deployLists, deployGets int
		for _, a := range f.fake.Actions() {
			switch a.GetResource() {
			case gvrs["configmaps"]:
				switch a.GetVerb() {
				case "list":
					cmLists++
				case "get":
					cmGets++
				}
			case gvrs["deployments"]:
				switch a.GetVerb() {
				case "list":
					deployLists++
				case "get":
					deployGets++
				}
			}
		}
		if cmLists != 1 || cmGets != 0 {
			t.Errorf("configmaps read actions: got %d list and %d get, want 1 list and 0 get", cmLists, cmGets)
		}
		if deployLists != 0 || deployGets != 1 {
			t.Errorf("deployments read actions: got %d list and %d get, want 0 list and 1 get", deployLists, deployGets)
		}

		cm1.SetOwnerReferences([]metav1.OwnerReference{ownerRef})
		cm2.SetOwnerReferences([]metav1.OwnerReference{ownerRef})
		deploy.SetOwnerReferences([]metav1.OwnerReference{ownerRef})
		setAppliedAnnotation(cm1)
		setAppliedAnnotation(cm2)
		setAppliedAnnotation(deploy)

		f.expectActions(
			k8stest.NewUpdateAction(gvrs["configmaps"], "foo1", cm1),
			k8stest.NewCreateAction(gvrs["configmaps"], "foo1", cm2),
			k8stest.NewCreateAction(gvrs["deployments"], "foo1", deploy),
		)
		f.verifyWriteActions()
	})

	t.Run("falls back to get when list fails", func(t *testing.T) {
		f := newFixture(t)
		s := f.newSynk()
		f.fake.PrependReactor("list", "deployments", func(action k8stest.Action) (bool, runtime.Object, error) {
			return true, nil, errors.New("list forbidden")
		})

		dp1 := newUnstructured("apps/v1", "Deployment", "foo1", "dp1")
		dp2 := newUnstructured("apps/v1", "Deployment", "foo1", "dp2")

		_, err := s.applyAll(t.Context(), set, &ApplyOptions{name: "test"},
			dp1.DeepCopy(),
			dp2.DeepCopy(),
		)
		if err != nil {
			t.Fatalf("applyAll() failed: %v", err)
		}

		var deployLists, deployGets int
		for _, a := range f.fake.Actions() {
			if a.GetResource() != gvrs["deployments"] {
				continue
			}
			switch a.GetVerb() {
			case "list":
				deployLists++
			case "get":
				deployGets++
			}
		}
		if deployLists != 1 || deployGets != 2 {
			t.Errorf("deployments read actions: got %d list and %d get, want 1 list and 2 get", deployLists, deployGets)
		}
	})

	t.Run("falls back to get on retry", func(t *testing.T) {
		f := newFixture(t)
		s := f.newSynk()
		f.fake.PrependReactor("create", "deployments", func(action k8stest.Action) (bool, runtime.Object, error) {
			ca := action.(k8stest.CreateAction)
			if ca.GetObject().(*unstructured.Unstructured).GetName() == "dp1" {
				return true, nil, k8serrors.NewResourceExpired("gone")
			}
			return false, nil, nil
		})

		dp1 := newUnstructured("apps/v1", "Deployment", "foo1", "dp1")
		dp2 := newUnstructured("apps/v1", "Deployment", "foo1", "dp2")

		_, err := s.applyAll(t.Context(), set, &ApplyOptions{name: "test"},
			dp1.DeepCopy(),
			dp2.DeepCopy(),
		)
		if err == nil {
			t.Fatalf("applyAll() succeeded unexpectedly")
		}

		var deployLists, deployGets int
		for _, a := range f.fake.Actions() {
			if a.GetResource() != gvrs["deployments"] {
				continue
			}
			switch a.GetVerb() {
			case "list":
				deployLists++
			case "get":
				deployGets++
			}
		}
		// Iteration 0 uses 1 List for both dp1 and dp2; iteration 1 retries failed dp1 using 1 Get.
		if deployLists != 1 || deployGets != 1 {
			t.Errorf("deployments read actions: got %d list and %d get, want 1 list and 1 get", deployLists, deployGets)
		}
	})
}

func TestSynk_skipsTestResources(t *testing.T) {
	ctx := t.Context()
	s := newFixture(t).newSynk()

	testPod := newUnstructured("v1", "Pod", "ns", "pod2")
	testPod.SetAnnotations(map[string]string{"helm.sh/hook": "test-success"})

	_, _, err := s.initialize(ctx, &ApplyOptions{name: "test"},
		newUnstructured("v1", "Pod", "ns", "pod1"),
		testPod,
	)
	if err != nil {
		t.Fatal(err)
	}
	got, err := s.client.Resource(resourceSetGVR).Get(ctx, "test.v1", metav1.GetOptions{})
	if err != nil {
		t.Fatal(err)
	}
	var want unstructured.Unstructured
	unmarshalYAML(t, &want, `
apiVersion: apps.cloudrobotics.com/v1alpha1
kind: ResourceSet
metadata:
  labels:
    name: test
  name: test.v1
spec:
  resources:
  - version: v1
    kind: Pod
    items:
    - name: pod1
      namespace: ns
status:
  phase: Pending
`)
	if !reflect.DeepEqual(want.Object["spec"], got.Object["spec"]) {
		t.Errorf("expected spec\n%v\nbut got\n%v", want.Object["spec"], got.Object["spec"])
	}
}

func TestSynk_deleteResourceSets(t *testing.T) {
	ctx := t.Context()
	nu := func(name, version string) *unstructured.Unstructured {
		u := newUnstructured("apps.cloudrobotics.com/v1alpha1", "ResourceSet", "", name+"."+version)
		u.SetLabels(map[string]string{"name": name})
		return u
	}
	f := newFixture(t)
	f.addObjects(
		nu("test", "v2"),
		nu("bad_name", ""),
		nu("other", "v3"),
		nu("test", "v4"),
		nu("test", "v7"),
		nu("test", "v8"),
	)
	synk := f.newSynk()

	err := synk.deleteResourceSets(ctx, "test", 7)
	if err != nil {
		t.Fatal(err)
	}
	f.expectActions(
		k8stest.NewRootDeleteAction(resourceSetGVR, "test.v2"),
		k8stest.NewRootDeleteAction(resourceSetGVR, "test.v4"),
	)
	f.verifyWriteActions()
}

func TestSynk_deleteFailedResourceSets(t *testing.T) {
	ctx := t.Context()
	nu := func(name, version string, failed bool) *unstructured.Unstructured {
		u := newUnstructured("apps.cloudrobotics.com/v1alpha1", "ResourceSet", "", name+"."+version)
		u.SetLabels(map[string]string{"name": name})
		if failed {
			unstructured.SetNestedField(u.Object, "Failed", "status", "phase")
		}
		return u
	}
	f := newFixture(t)
	f.addObjects(
		nu("test", "v2", true),
		nu("test", "v4", false),
		nu("test", "v6", true),
		nu("test", "v7", true),
		nu("test", "v8", true),
	)
	synk := f.newSynk()

	err := synk.deleteFailedResourceSets(ctx, "test", 7)
	if err != nil {
		t.Fatal(err)
	}
	f.expectActions(
		k8stest.NewRootDeleteAction(resourceSetGVR, "test.v2"),
		k8stest.NewRootDeleteAction(resourceSetGVR, "test.v6"),
	)
	f.verifyWriteActions()
}

func TestSynk_next(t *testing.T) {
	ctx := t.Context()
	nu := func(name, version string) *unstructured.Unstructured {
		u := newUnstructured("apps.cloudrobotics.com/v1alpha1", "ResourceSet", "", name+"."+version)
		u.SetLabels(map[string]string{"name": name})
		return u
	}
	f := newFixture(t)
	f.addObjects(
		nu("test", "v2"),
		nu("other", "v9"),
		nu("test", "v5"),
	)
	synk := f.newSynk()

	version, err := synk.next(ctx, "test")
	if err != nil {
		t.Fatal(err)
	}
	if version != 6 {
		t.Errorf("next() = %d, want 6", version)
	}
	actions := f.fake.Actions()
	if len(actions) != 1 {
		t.Fatalf("expected 1 action, got %d", len(actions))
	}
	listAction, ok := actions[0].(k8stest.ListAction)
	if !ok {
		t.Fatalf("expected ListAction, got %T", actions[0])
	}
	if got := listAction.GetListRestrictions().Labels.String(); got != "name=test" {
		t.Errorf("List label selector = %q, want %q", got, "name=test")
	}
}

func TestSynk_Apply_ResourceSetCleanup(t *testing.T) {
	nu := func(name, version string, failed bool) *unstructured.Unstructured {
		u := newUnstructured("apps.cloudrobotics.com/v1alpha1", "ResourceSet", "", name+"."+version)
		u.SetLabels(map[string]string{"name": name})
		if failed {
			unstructured.SetNestedField(u.Object, "Failed", "status", "phase")
		} else {
			unstructured.SetNestedField(u.Object, "Settled", "status", "phase")
		}
		return u
	}

	t.Run("success deletes all older ResourceSets in one list", func(t *testing.T) {
		f := newFixture(t)
		f.addObjects(
			nu("test", "v1", false),
			nu("test", "v2", true),
		)
		s := f.newSynk()

		_, err := s.Apply(t.Context(), "test", &ApplyOptions{Namespace: "ns"},
			newUnstructured("v1", "Pod", "ns", "pod1"),
		)
		if err != nil {
			t.Fatalf("Apply() failed: %v", err)
		}

		var rsLists int
		var deleted []string
		for _, a := range f.fake.Actions() {
			if a.GetResource() != resourceSetGVR {
				continue
			}
			switch v := a.(type) {
			case k8stest.ListAction:
				rsLists++
			case k8stest.DeleteAction:
				deleted = append(deleted, v.GetName())
			}
		}
		// 1 List in next() + 1 List in deleteResourceSets() (no redundant List in deleteFailedResourceSets()).
		if rsLists != 2 {
			t.Errorf("expected 2 ResourceSet List calls, got %d", rsLists)
		}
		wantDeleted := []string{"test.v1", "test.v2"}
		if !reflect.DeepEqual(deleted, wantDeleted) {
			t.Errorf("deleted ResourceSets = %v, want %v", deleted, wantDeleted)
		}
	})

	t.Run("failure deletes only older failed ResourceSets", func(t *testing.T) {
		f := newFixture(t)
		f.addObjects(
			nu("test", "v1", false),
			nu("test", "v2", true),
		)
		s := f.newSynk()
		f.fake.PrependReactor("create", "pods", func(action k8stest.Action) (bool, runtime.Object, error) {
			return true, nil, errors.New("permanent failure")
		})

		_, err := s.Apply(t.Context(), "test", &ApplyOptions{Namespace: "ns"},
			newUnstructured("v1", "Pod", "ns", "pod1"),
		)
		if err == nil {
			t.Fatalf("Apply() succeeded unexpectedly")
		}

		var rsLists int
		var deleted []string
		for _, a := range f.fake.Actions() {
			if a.GetResource() != resourceSetGVR {
				continue
			}
			switch v := a.(type) {
			case k8stest.ListAction:
				rsLists++
			case k8stest.DeleteAction:
				deleted = append(deleted, v.GetName())
			}
		}
		// 1 List in next() + 1 List in deleteFailedResourceSets().
		if rsLists != 2 {
			t.Errorf("expected 2 ResourceSet List calls, got %d", rsLists)
		}
		wantDeleted := []string{"test.v2"}
		if !reflect.DeepEqual(deleted, wantDeleted) {
			t.Errorf("deleted ResourceSets = %v, want %v", deleted, wantDeleted)
		}
	})
}

func TestSynk_populateNamespaces(t *testing.T) {
	f := newFixture(t)
	s := f.newSynk()

	var (
		ns1  = newUnstructured("v1", "Namespace", "", "ns1")
		pod1 = newUnstructured("v1", "Pod", "ns1", "pod1")
		pod2 = newUnstructured("v1", "Pod", "", "pod1")
		cr1  = newUnstructured("example.org/v1", "Example", "", "pod1")
	)
	var exampleCRD unstructured.Unstructured
	unmarshalYAML(t, &exampleCRD, `
apiVersion: apiextensions.k8s.io/v1
kind: CustomResourceDefition
metadata:
  name: examples.example.org
spec:
  group: example.org
  names:
    kind: Example
  scope: Namespaced
  versions:
    - name: v1
      served: true
      storage: true`)

	if err := s.populateNamespaces(
		t.Context(),
		"ns2",
		[]*unstructured.Unstructured{&exampleCRD},
		ns1, pod1, pod2, cr1,
	); err != nil {
		t.Fatal(err)
	}

	if ns1.GetNamespace() != "" {
		t.Errorf("unexpected namespace %q added to ns1", ns1.GetNamespace())
	}
	if pod1.GetNamespace() != "ns1" {
		t.Errorf("unexpected namespace change to %q to pod1", pod1.GetNamespace())
	}
	if pod2.GetNamespace() != "ns2" {
		t.Errorf("unexpected namesapce %q on pod2", pod2.GetNamespace())
	}
	if cr1.GetNamespace() != "ns2" {
		t.Errorf("unexpected namesapce %q on cr1", cr1.GetNamespace())
	}
}

func TestSynk_skipLastAppliedAnnotationForLargeResource(t *testing.T) {
	yaml := `
apiVersion: v1
kind: ConfigMap
metadata:
  namespace: foo1
  name: cm1
data:
  foo1: ` + strings.Repeat("x", totalAnnotationSizeLimitB)

	var cmLarge unstructured.Unstructured
	unmarshalYAML(t, &cmLarge, yaml)
	if err := setAppliedAnnotation(&cmLarge); err == nil {
		t.Errorf("expected error setting large annotation, got nil")
	}

	if applied := getAppliedAnnotation(&cmLarge); len(applied) > 0 {
		t.Errorf("expected no last-applied annotation set, but got %v", applied)
	}
}

func TestSynk_validateNamespace(t *testing.T) {
	tests := []struct {
		desc        string
		namespace   string
		optsNs      string
		annotations map[string]string
		wantErr     bool
	}{
		{
			desc:      "empty namespace is allowed",
			namespace: "",
			optsNs:    "my-ns",
			wantErr:   false,
		},
		{
			desc:      "kube-system is allowed",
			namespace: "kube-system",
			optsNs:    "my-ns",
			wantErr:   false,
		},
		{
			desc:      "default is allowed",
			namespace: "default",
			optsNs:    "my-ns",
			wantErr:   false,
		},
		{
			desc:      "envoy-gateway-system is allowed",
			namespace: "envoy-gateway-system",
			optsNs:    "my-ns",
			wantErr:   false,
		},
		{
			desc:      "opts namespace is allowed",
			namespace: "my-ns",
			optsNs:    "my-ns",
			wantErr:   false,
		},
		{
			desc:      "other namespace is not allowed",
			namespace: "other-ns",
			optsNs:    "my-ns",
			wantErr:   true,
		},
		{
			desc:        "other namespace is allowed with allow-cross-namespace annotation",
			namespace:   "other-ns",
			optsNs:      "my-ns",
			annotations: map[string]string{AnnotationAllowCrossNamespace: "true"},
			wantErr:     false,
		},
		{
			desc:        "other namespace is not allowed when allow-cross-namespace annotation is false",
			namespace:   "other-ns",
			optsNs:      "my-ns",
			annotations: map[string]string{AnnotationAllowCrossNamespace: "false"},
			wantErr:     true,
		},
		{
			desc:      "custom ns not allowed-listed via optsNs",
			namespace: "my-ns",
			optsNs:    "",
			wantErr:   true,
		},
	}

	for _, tc := range tests {
		t.Run(tc.desc, func(t *testing.T) {
			r := newUnstructured("v1", "Pod", tc.namespace, "pod1")
			if tc.annotations != nil {
				r.SetAnnotations(tc.annotations)
			}
			err := validateNamespace(r, tc.optsNs)
			if (err != nil) != tc.wantErr {
				t.Errorf("validateNamespace() error = %v, wantErr %v", err, tc.wantErr)
			}
		})
	}
}

func TestSynk_canReplace(t *testing.T) {
	tests := []struct {
		desc     string
		kind     string
		err      error
		expected bool
	}{
		{
			desc:     "Deployment immutable field error",
			kind:     "Deployment",
			err:      errors.New("field is immutable"),
			expected: true,
		},
		{
			desc:     "Service may not change once set error",
			kind:     "Service",
			err:      errors.New("may not change once set"),
			expected: true,
		},
		{
			desc:     "Job immutable field error",
			kind:     "Job",
			err:      errors.New("field is immutable"),
			expected: true,
		},
		{
			desc:     "ValidatingWebhookConfiguration must be specified for an update",
			kind:     "ValidatingWebhookConfiguration",
			err:      errors.New("must be specified for an update"),
			expected: true,
		},
		{
			desc:     "Pod immutable field error (not replaceable)",
			kind:     "Pod",
			err:      errors.New("field is immutable"),
			expected: false,
		},
		{
			desc:     "Random error",
			kind:     "Deployment",
			err:      errors.New("random error"),
			expected: false,
		},
	}

	for _, tc := range tests {
		t.Run(tc.desc, func(t *testing.T) {
			r := newUnstructured("v1", tc.kind, "ns", "name")
			if got := canReplace(r, tc.err); got != tc.expected {
				t.Errorf("canReplace() = %v, want %v", got, tc.expected)
			}
		})
	}
}

func TestSynk_Delete(t *testing.T) {
	ctx := t.Context()
	f := newFixture(t)
	s := f.newSynk()

	err := s.Delete(ctx, "test")
	if err != nil {
		t.Fatal(err)
	}

	f.expectActions(
		k8stest.NewRootDeleteCollectionAction(resourceSetGVR, metav1.ListOptions{LabelSelector: "name=test"}),
	)
	// DeleteCollectionAction doesn't match DeleteActionImpl in sprintAction.
	// But it should be in writes.
	writes := filterReadActions(f.fake.Actions())
	if len(writes) != 1 {
		t.Errorf("expected 1 write action, got %d", len(writes))
	}
}

func TestSynk_Init(t *testing.T) {
	f := newFixture(t)
	s := f.newSynk()

	// Mock discovery to return the ResourceSet CRD after it's created.
	// Init calls crdAvailable which calls ServerResourcesForGroupVersion.
	f.discovery.resources["apps.cloudrobotics.com/v1alpha1"] = &metav1.APIResourceList{
		GroupVersion: "apps.cloudrobotics.com/v1alpha1",
		APIResources: []metav1.APIResource{
			{Name: "resourcesets", Kind: "ResourceSet"},
		},
	}

	err := s.Init(t.Context())
	if err != nil {
		t.Fatal(err)
	}

	// Should have called Create for the CRD.
	writes := filterReadActions(f.fake.Actions())
	found := false
	for _, a := range writes {
		if ca, ok := a.(k8stest.CreateAction); ok && ca.GetResource().Resource == "customresourcedefinitions" {
			found = true
			break
		}
	}
	if !found {
		t.Errorf("expected Create action for CustomResourceDefinition")
	}
}

func newUnstructured(apiVersion, kind, namespace, name string) *unstructured.Unstructured {
	var u unstructured.Unstructured
	u.SetAPIVersion(apiVersion)
	u.SetKind(kind)
	u.SetNamespace(namespace)
	u.SetName(name)
	return &u
}

func unmarshalYAML(t *testing.T, v interface{}, s string) {
	t.Helper()
	if err := yaml.Unmarshal([]byte(strings.TrimSpace(s)), v); err != nil {
		t.Fatal(err)
	}
}

func toUnstructured(t *testing.T, o runtime.Object) *unstructured.Unstructured {
	t.Helper()
	var u unstructured.Unstructured
	if err := convert(o, &u); err != nil {
		t.Fatal(err)
	}
	return &u
}

// filterReadActions drops read-only actions that we don't care about to verify
// the correct behavior.
func filterReadActions(actions []k8stest.Action) (ret []k8stest.Action) {
	for _, a := range actions {
		if v := a.GetVerb(); v == "watch" || v == "list" || v == "get" {
			continue
		}
		ret = append(ret, a)
	}
	return ret
}

func sprintAction(a k8stest.Action) string {
	switch v := a.(type) {
	case k8stest.DeleteActionImpl:
		return fmt.Sprintf("DELETE %s/%s %s/%s", v.Resource, v.Subresource, v.Namespace, v.Name)
	case k8stest.CreateActionImpl:
		return fmt.Sprintf("CREATE %s/%s %s/%s: %v", v.Resource, v.Subresource, v.Namespace, v.Name, v.Object.(*unstructured.Unstructured))
	case k8stest.UpdateActionImpl:
		return fmt.Sprintf("UPDATE %s/%s %s: %v", v.Resource, v.Subresource, v.Namespace, v.Object.(*unstructured.Unstructured))
	case k8stest.PatchActionImpl:
		return fmt.Sprintf("PATCH %s/%s %s/%s: %s %s", v.Resource, v.Subresource, v.Namespace, v.Name, v.PatchType, v.Patch)
	default:
		return fmt.Sprintf("<UNKNOWN ACTION %T>", a)
	}
}

func Test_resourceSetName(t *testing.T) {
	cases := []struct {
		name    string
		version int
		want    string
	}{
		{"app", 1, "app.v1"},
		{"my.chart", 42, "my.chart.v42"},
	}
	for _, c := range cases {
		if got := resourceSetName(c.name, c.version); got != c.want {
			t.Errorf("resourceSetName(%q, %d) = %q, want %q", c.name, c.version, got, c.want)
		}
	}
}

func Test_decodeResourceSetName(t *testing.T) {
	cases := []struct {
		input       string
		wantName    string
		wantVersion int
		wantOk      bool
	}{
		{"app.v1", "app", 1, true},
		{"my.chart.name.v42", "my.chart.name", 42, true},
		{"app.v0", "app", 0, true},
		{"app.v12345", "app", 12345, true},
		{"app", "", 0, false},
		{"app.v", "", 0, false},
		{"app.v-1", "", 0, false},
		{"app.vabc", "", 0, false},
		{".v1", "", 0, false},
		{"app.v999999999999999999999999999999999999999", "", 0, false},
	}
	for _, c := range cases {
		gotName, gotVersion, gotOk := decodeResourceSetName(c.input)
		if gotName != c.wantName || gotVersion != c.wantVersion || gotOk != c.wantOk {
			t.Errorf("decodeResourceSetName(%q) = (%q, %d, %v), want (%q, %d, %v)",
				c.input, gotName, gotVersion, gotOk, c.wantName, c.wantVersion, c.wantOk)
		}
	}
}

func TestSyncPodSpecServiceAccount(t *testing.T) {
	tests := []struct {
		name        string
		input       map[string]any
		want        map[string]any
		wantChanged bool
	}{
		{
			name:        "neither set",
			input:       map[string]any{"containers": []any{}},
			want:        map[string]any{"containers": []any{}},
			wantChanged: false,
		},
		{
			name:        "only serviceAccountName set",
			input:       map[string]any{"serviceAccountName": "foo"},
			want:        map[string]any{"serviceAccountName": "foo", "serviceAccount": "foo"},
			wantChanged: true,
		},
		{
			name:        "only serviceAccount set",
			input:       map[string]any{"serviceAccount": "foo"},
			want:        map[string]any{"serviceAccount": "foo", "serviceAccountName": "foo"},
			wantChanged: true,
		},
		{
			name:        "both matching",
			input:       map[string]any{"serviceAccountName": "foo", "serviceAccount": "foo"},
			want:        map[string]any{"serviceAccountName": "foo", "serviceAccount": "foo"},
			wantChanged: false,
		},
		{
			name:        "conflicting non-empty aligns to serviceAccountName",
			input:       map[string]any{"serviceAccountName": "new-sa", "serviceAccount": "old-sa"},
			want:        map[string]any{"serviceAccountName": "new-sa", "serviceAccount": "new-sa"},
			wantChanged: true,
		},
		{
			name:        "serviceAccountName set with empty serviceAccount",
			input:       map[string]any{"serviceAccountName": "foo", "serviceAccount": ""},
			want:        map[string]any{"serviceAccountName": "foo", "serviceAccount": "foo"},
			wantChanged: true,
		},
		{
			name:        "serviceAccountName set with nil serviceAccount",
			input:       map[string]any{"serviceAccountName": "foo", "serviceAccount": nil},
			want:        map[string]any{"serviceAccountName": "foo", "serviceAccount": "foo"},
			wantChanged: true,
		},
		{
			name:        "empty serviceAccountName with serviceAccount set",
			input:       map[string]any{"serviceAccountName": "", "serviceAccount": "bar"},
			want:        map[string]any{"serviceAccountName": "bar", "serviceAccount": "bar"},
			wantChanged: true,
		},
		{
			name:        "nil serviceAccountName with serviceAccount set",
			input:       map[string]any{"serviceAccountName": nil, "serviceAccount": "bar"},
			want:        map[string]any{"serviceAccountName": "bar", "serviceAccount": "bar"},
			wantChanged: true,
		},
		{
			name:        "only serviceAccountName empty string",
			input:       map[string]any{"serviceAccountName": ""},
			want:        map[string]any{"serviceAccountName": "", "serviceAccount": ""},
			wantChanged: true,
		},
		{
			name:        "only serviceAccount empty string",
			input:       map[string]any{"serviceAccount": ""},
			want:        map[string]any{"serviceAccountName": "", "serviceAccount": ""},
			wantChanged: true,
		},
		{
			name:        "only serviceAccountName nil",
			input:       map[string]any{"serviceAccountName": nil},
			want:        map[string]any{"serviceAccountName": nil, "serviceAccount": nil},
			wantChanged: true,
		},
		{
			name:        "only serviceAccount nil",
			input:       map[string]any{"serviceAccount": nil},
			want:        map[string]any{"serviceAccountName": nil, "serviceAccount": nil},
			wantChanged: true,
		},
		{
			name:        "both empty string",
			input:       map[string]any{"serviceAccountName": "", "serviceAccount": ""},
			want:        map[string]any{"serviceAccountName": "", "serviceAccount": ""},
			wantChanged: false,
		},
		{
			name:        "both nil",
			input:       map[string]any{"serviceAccountName": nil, "serviceAccount": nil},
			want:        map[string]any{"serviceAccountName": nil, "serviceAccount": nil},
			wantChanged: false,
		},
		{
			name:        "mixed empty string and nil",
			input:       map[string]any{"serviceAccountName": "", "serviceAccount": nil},
			want:        map[string]any{"serviceAccountName": "", "serviceAccount": nil},
			wantChanged: false,
		},
		{
			name:        "malformed non-string serviceAccountName untouched",
			input:       map[string]any{"serviceAccountName": 123},
			want:        map[string]any{"serviceAccountName": 123},
			wantChanged: false,
		},
		{
			name:        "malformed non-string serviceAccount untouched",
			input:       map[string]any{"serviceAccountName": "foo", "serviceAccount": true},
			want:        map[string]any{"serviceAccountName": "foo", "serviceAccount": true},
			wantChanged: false,
		},
	}

	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			gotChanged := syncPodSpecServiceAccount(tc.input, nil)
			if gotChanged != tc.wantChanged {
				t.Errorf("syncPodSpecServiceAccount() changed = %v, want %v", gotChanged, tc.wantChanged)
			}
			if !reflect.DeepEqual(tc.input, tc.want) {
				t.Errorf("syncPodSpecServiceAccount() result = %#v, want %#v", tc.input, tc.want)
			}
		})
	}
}

func TestNormalizeOriginalPodSpecServiceAccount(t *testing.T) {
	t.Run("preserves large integers without float64 precision loss", func(t *testing.T) {
		raw := []byte(`{"spec":{"bigInt":9007199254740993,"template":{"spec":{"serviceAccountName":"foo"}}}}`)
		got := string(normalizeOriginalPodSpecServiceAccount(raw, schema.GroupKind{Group: "apps", Kind: "Deployment"}))
		if !strings.Contains(got, "9007199254740993") {
			t.Errorf("lost integer precision: %s", got)
		}
		if !strings.Contains(got, `"serviceAccount":"foo"`) {
			t.Errorf("expected serviceAccount mirrored, got: %s", got)
		}
	})

	t.Run("handles unicode-escaped key", func(t *testing.T) {
		raw := []byte(`{"spec":{"template":{"spec":{"\u0073erviceAccountName":"foo"}}}}`)
		got := normalizeOriginalPodSpecServiceAccount(raw, schema.GroupKind{Group: "apps", Kind: "Deployment"})
		var obj map[string]any
		if err := json.Unmarshal(got, &obj); err != nil {
			t.Fatal(err)
		}
		sa, _, _ := unstructured.NestedString(obj, "spec", "template", "spec", "serviceAccount")
		if sa != "foo" {
			t.Errorf("expected mirrored serviceAccount=foo, got %q (%s)", sa, got)
		}
	})

	t.Run("ignores unknown kinds and non-core groups", func(t *testing.T) {
		raw := []byte(`{"spec":{"template":{"spec":{"serviceAccountName":"foo"}}}}`)
		for _, gk := range []schema.GroupKind{
			{Group: "", Kind: "ConfigMap"},
			{Group: "example.com", Kind: "Deployment"},
			{Group: "apps.cloudrobotics.com", Kind: "AppRollout"},
		} {
			got := normalizeOriginalPodSpecServiceAccount(raw, gk)
			if string(got) != string(raw) {
				t.Errorf("expected %v unchanged, got %s", gk, got)
			}
		}
	})

	t.Run("passes through invalid or non-object JSON unchanged", func(t *testing.T) {
		gk := schema.GroupKind{Group: "apps", Kind: "Deployment"}
		for _, raw := range [][]byte{
			nil,
			[]byte(``),
			[]byte(`{invalid serviceAccountName`),
			[]byte(`null`),
			[]byte(`["serviceAccountName"]`),
		} {
			got := normalizeOriginalPodSpecServiceAccount(raw, gk)
			if string(got) != string(raw) {
				t.Errorf("for %q: got %q, want unchanged", raw, got)
			}
		}
	})

	t.Run("handles all built-in PodSpec kinds", func(t *testing.T) {
		cases := []struct {
			gk   schema.GroupKind
			raw  string
			path []string
		}{
			{schema.GroupKind{Group: "", Kind: "Pod"}, `{"spec":{"serviceAccountName":"foo"}}`, []string{"spec"}},
			{schema.GroupKind{Group: "", Kind: "PodTemplate"}, `{"template":{"spec":{"serviceAccountName":"foo"}}}`, []string{"template", "spec"}},
			{schema.GroupKind{Group: "", Kind: "ReplicationController"}, `{"spec":{"template":{"spec":{"serviceAccountName":"foo"}}}}`, []string{"spec", "template", "spec"}},
			{schema.GroupKind{Group: "apps", Kind: "Deployment"}, `{"spec":{"template":{"spec":{"serviceAccountName":"foo"}}}}`, []string{"spec", "template", "spec"}},
			{schema.GroupKind{Group: "apps", Kind: "DaemonSet"}, `{"spec":{"template":{"spec":{"serviceAccountName":"foo"}}}}`, []string{"spec", "template", "spec"}},
			{schema.GroupKind{Group: "apps", Kind: "ReplicaSet"}, `{"spec":{"template":{"spec":{"serviceAccountName":"foo"}}}}`, []string{"spec", "template", "spec"}},
			{schema.GroupKind{Group: "apps", Kind: "StatefulSet"}, `{"spec":{"template":{"spec":{"serviceAccountName":"foo"}}}}`, []string{"spec", "template", "spec"}},
			{schema.GroupKind{Group: "extensions", Kind: "Deployment"}, `{"spec":{"template":{"spec":{"serviceAccountName":"foo"}}}}`, []string{"spec", "template", "spec"}},
			{schema.GroupKind{Group: "extensions", Kind: "DaemonSet"}, `{"spec":{"template":{"spec":{"serviceAccountName":"foo"}}}}`, []string{"spec", "template", "spec"}},
			{schema.GroupKind{Group: "extensions", Kind: "ReplicaSet"}, `{"spec":{"template":{"spec":{"serviceAccountName":"foo"}}}}`, []string{"spec", "template", "spec"}},
			{schema.GroupKind{Group: "batch", Kind: "Job"}, `{"spec":{"template":{"spec":{"serviceAccountName":"foo"}}}}`, []string{"spec", "template", "spec"}},
			{schema.GroupKind{Group: "batch", Kind: "CronJob"}, `{"spec":{"jobTemplate":{"spec":{"template":{"spec":{"serviceAccountName":"foo"}}}}}}`, []string{"spec", "jobTemplate", "spec", "template", "spec"}},
		}
		for _, tc := range cases {
			got := normalizeOriginalPodSpecServiceAccount([]byte(tc.raw), tc.gk)
			var obj map[string]any
			if err := json.Unmarshal(got, &obj); err != nil {
				t.Fatalf("%v: unmarshal failed: %v", tc.gk, err)
			}
			sa, ok, err := unstructured.NestedString(obj, append(tc.path, "serviceAccount")...)
			if err != nil || !ok || sa != "foo" {
				t.Errorf("%v: serviceAccount = (%q, %v, %v), want (\"foo\", true, nil)", tc.gk, sa, ok, err)
			}
		}
	})
}

// applyServerPodSpecDefaults mimics kube-apiserver's SetDefaults_PodSpec on a
// PodSpec map, mirroring serviceAccountName <-> serviceAccount and dropping
// empty/null fields just like Kubernetes JSON omitempty serialization.
func applyServerPodSpecDefaults(u *unstructured.Unstructured) {
	path := podSpecPath(u.GroupVersionKind().GroupKind())
	if len(path) == 0 {
		return
	}
	podSpec, ok := nestedMapRef(u.Object, path)
	if !ok {
		return
	}
	saName, _ := podSpec["serviceAccountName"].(string)
	sa, _ := podSpec["serviceAccount"].(string)
	if len(saName) == 0 {
		saName = sa
	}
	sa = saName
	if saName == "" {
		delete(podSpec, "serviceAccountName")
		delete(podSpec, "serviceAccount")
	} else {
		podSpec["serviceAccountName"] = saName
		podSpec["serviceAccount"] = sa
	}
	// Mimic Kubernetes ServiceAccount admission controller for Pods.
	if u.GroupVersionKind().GroupKind() == (schema.GroupKind{Group: "", Kind: "Pod"}) {
		if saName == "" {
			podSpec["serviceAccountName"] = "default"
			podSpec["serviceAccount"] = "default"
		}
	}
}

// installApiserverPatchReactor adds reactors to a fake dynamic client that
// simulate kube-apiserver's StrategicMergePatch, Update, Create, and
// SetDefaults_PodSpec behavior so tests catch any regression where
// SetDefaults_PodSpec silently restores serviceAccountName from serviceAccount.
func installApiserverPatchReactor(t *testing.T, client *dynamicfake.FakeDynamicClient, immutableJobs bool) *[]string {
	t.Helper()
	var patches []string

	client.Fake.PrependReactor("create", "*", func(action k8stest.Action) (bool, runtime.Object, error) {
		ca := action.(k8stest.CreateAction)
		u := ca.GetObject().(*unstructured.Unstructured).DeepCopy()
		applyServerPodSpecDefaults(u)
		if err := client.Tracker().Create(ca.GetResource(), u, ca.GetNamespace()); err != nil {
			return true, nil, err
		}
		return true, u.DeepCopy(), nil
	})

	client.Fake.PrependReactor("update", "*", func(action k8stest.Action) (bool, runtime.Object, error) {
		ua := action.(k8stest.UpdateAction)
		u := ua.GetObject().(*unstructured.Unstructured).DeepCopy()
		applyServerPodSpecDefaults(u)
		if err := client.Tracker().Update(ua.GetResource(), u, ua.GetNamespace()); err != nil {
			return true, nil, err
		}
		return true, u.DeepCopy(), nil
	})

	client.Fake.PrependReactor("patch", "*", func(action k8stest.Action) (bool, runtime.Object, error) {
		pa := action.(k8stest.PatchAction)
		patches = append(patches, string(pa.GetPatch()))
		existing, err := client.Tracker().Get(pa.GetResource(), pa.GetNamespace(), pa.GetName())
		if err != nil {
			return true, nil, err
		}
		origU := existing.(*unstructured.Unstructured).DeepCopy()
		origBytes, err := origU.MarshalJSON()
		if err != nil {
			return true, nil, err
		}
		if pa.GetPatchType() != types.StrategicMergePatchType {
			return true, nil, fmt.Errorf("unexpected patch type %s", pa.GetPatchType())
		}
		gvk := origU.GroupVersionKind()
		typed, err := scheme.Scheme.New(gvk)
		if err != nil {
			return true, nil, fmt.Errorf("scheme.New(%v): %w", gvk, err)
		}
		patchMeta, err := strategicpatch.NewPatchMetaFromStruct(typed)
		if err != nil {
			return true, nil, err
		}
		mergedBytes, err := strategicpatch.StrategicMergePatchUsingLookupPatchMeta(origBytes, pa.GetPatch(), patchMeta)
		if err != nil {
			return true, nil, err
		}
		var merged unstructured.Unstructured
		if err := merged.UnmarshalJSON(mergedBytes); err != nil {
			return true, nil, err
		}
		// Mirror kube-apiserver pipeline: SetDefaults_PodSpec runs BEFORE validation.
		applyServerPodSpecDefaults(&merged)
		if immutableJobs && gvk.Kind == "Job" {
			oldTmpl, _, _ := unstructured.NestedMap(origU.Object, "spec", "template")
			newTmpl, _, _ := unstructured.NestedMap(merged.Object, "spec", "template")
			if !reflect.DeepEqual(oldTmpl, newTmpl) {
				return true, nil, fmt.Errorf(`Job.batch %q is invalid: spec.template: field is immutable`, pa.GetName())
			}
		}
		if err := client.Tracker().Update(pa.GetResource(), &merged, pa.GetNamespace()); err != nil {
			return true, nil, err
		}
		return true, merged.DeepCopy(), nil
	})
	return &patches
}

func makeWorkload(apiVersion, kind, name, saName, sa string, explicitEmptySAName bool) *unstructured.Unstructured {
	u := newUnstructured(apiVersion, kind, "app-ns", name)
	podSpec := map[string]any{
		"containers": []any{
			map[string]any{"name": "app", "image": "busybox:latest"},
		},
	}
	if saName != "" || explicitEmptySAName {
		podSpec["serviceAccountName"] = saName
	}
	if sa != "" {
		podSpec["serviceAccount"] = sa
	}
	switch kind {
	case "Pod":
		u.Object["spec"] = podSpec
	case "PodTemplate":
		u.Object["template"] = map[string]any{"spec": podSpec}
	case "CronJob":
		u.Object["spec"] = map[string]any{
			"schedule": "* * * * *",
			"jobTemplate": map[string]any{
				"spec": map[string]any{
					"template": map[string]any{"spec": podSpec},
				},
			},
		}
	default:
		u.Object["spec"] = map[string]any{
			"template": map[string]any{
				"metadata": map[string]any{"labels": map[string]any{"app": name}},
				"spec":     podSpec,
			},
		}
	}
	return u
}

func newServerBackedSynk(t *testing.T, immutableJobs bool, initial ...*unstructured.Unstructured) (*Synk, *dynamicfake.FakeDynamicClient, *[]string) {
	t.Helper()
	sc := runtime.NewScheme()
	scheme.AddToScheme(sc)
	apps.AddToScheme(sc)
	apiextensions.AddToScheme(sc)

	disc := &fakeCachedDiscoveryClient{resources: map[string]*metav1.APIResourceList{
		"v1": {
			GroupVersion: "v1",
			APIResources: []metav1.APIResource{
				{Name: "pods", Kind: "Pod", Namespaced: true},
				{Name: "podtemplates", Kind: "PodTemplate", Namespaced: true},
				{Name: "replicationcontrollers", Kind: "ReplicationController", Namespaced: true},
				{Name: "serviceaccounts", Kind: "ServiceAccount", Namespaced: true},
			},
		},
		"apps/v1": {
			GroupVersion: "apps/v1",
			APIResources: []metav1.APIResource{
				{Name: "deployments", Kind: "Deployment", Namespaced: true},
				{Name: "daemonsets", Kind: "DaemonSet", Namespaced: true},
				{Name: "statefulsets", Kind: "StatefulSet", Namespaced: true},
				{Name: "replicasets", Kind: "ReplicaSet", Namespaced: true},
			},
		},
		"batch/v1": {
			GroupVersion: "batch/v1",
			APIResources: []metav1.APIResource{
				{Name: "jobs", Kind: "Job", Namespaced: true},
				{Name: "cronjobs", Kind: "CronJob", Namespaced: true},
			},
		},
	}}

	objs := make([]runtime.Object, len(initial))
	for i, o := range initial {
		objs[i] = o
	}
	client := dynamicfake.NewSimpleDynamicClient(sc, objs...)
	s := New(client, disc)
	s.mapper = testrestmapper.TestOnlyStaticRESTMapper(sc)
	s.resetMapper = func() {}
	s.workers = 1
	patches := installApiserverPatchReactor(t, client, immutableJobs)
	return s, client, patches
}

func assertServiceAccountFields(t *testing.T, u *unstructured.Unstructured, wantSAName, wantSA string) {
	t.Helper()
	path := podSpecPath(u.GroupVersionKind().GroupKind())
	podSpec, ok := nestedMapRef(u.Object, path)
	if !ok {
		t.Fatalf("missing podSpec at %v in %v", path, u.Object)
	}
	gotSAName, _ := podSpec["serviceAccountName"].(string)
	gotSA, _ := podSpec["serviceAccount"].(string)
	if gotSAName != wantSAName || gotSA != wantSA {
		t.Errorf("%s %s: got (serviceAccountName=%q, serviceAccount=%q), want (%q, %q)",
			u.GetKind(), u.GetName(), gotSAName, gotSA, wantSAName, wantSA)
	}
}

func assertLastAppliedSAFields(t *testing.T, u *unstructured.Unstructured, wantSAName, wantSA string, wantPresent bool) {
	t.Helper()
	raw := getAppliedAnnotation(u)
	var la map[string]any
	if err := json.Unmarshal(raw, &la); err != nil {
		t.Fatalf("unmarshal last-applied-configuration: %v", err)
	}
	path := podSpecPath(u.GroupVersionKind().GroupKind())
	podSpec, ok := nestedMapRef(la, path)
	if !ok {
		t.Fatalf("missing podSpec at %v in last-applied-configuration", path)
	}
	saName, hasSAName := podSpec["serviceAccountName"]
	sa, hasSA := podSpec["serviceAccount"]
	if !wantPresent {
		if hasSAName || hasSA {
			t.Errorf("last-applied-configuration expected neither SA field present, got serviceAccountName=%v (%v) serviceAccount=%v (%v)", saName, hasSAName, sa, hasSA)
		}
		return
	}
	if !hasSAName || !hasSA || saName != wantSAName || sa != wantSA {
		t.Errorf("last-applied-configuration got (serviceAccountName=%v present=%v, serviceAccount=%v present=%v), want (%q, %q)",
			saName, hasSAName, sa, hasSA, wantSAName, wantSA)
	}
}

func TestSynk_serviceAccountUpgradeAndDowngrade(t *testing.T) {
	ctx := t.Context()
	opts := &ApplyOptions{Namespace: "app-ns"}

	workloads := []struct {
		apiVersion string
		kind       string
		gvr        schema.GroupVersionResource
	}{
		{"apps/v1", "Deployment", schema.GroupVersionResource{Group: "apps", Version: "v1", Resource: "deployments"}},
		{"apps/v1", "DaemonSet", schema.GroupVersionResource{Group: "apps", Version: "v1", Resource: "daemonsets"}},
		{"apps/v1", "StatefulSet", schema.GroupVersionResource{Group: "apps", Version: "v1", Resource: "statefulsets"}},
		{"apps/v1", "ReplicaSet", schema.GroupVersionResource{Group: "apps", Version: "v1", Resource: "replicasets"}},
		{"v1", "ReplicationController", schema.GroupVersionResource{Version: "v1", Resource: "replicationcontrollers"}},
		{"v1", "PodTemplate", schema.GroupVersionResource{Version: "v1", Resource: "podtemplates"}},
		{"batch/v1", "CronJob", schema.GroupVersionResource{Group: "batch", Version: "v1", Resource: "cronjobs"}},
	}

	for _, w := range workloads {
		t.Run(w.kind+"/full_lifecycle", func(t *testing.T) {
			s, client, patches := newServerBackedSynk(t, false)
			getLive := func() *unstructured.Unstructured {
				u, err := client.Resource(w.gvr).Namespace("app-ns").Get(ctx, "wl", metav1.GetOptions{})
				if err != nil {
					t.Fatal(err)
				}
				return u
			}

			// 1. Create without SA.
			if _, err := s.Apply(ctx, "chart", opts, makeWorkload(w.apiVersion, w.kind, "wl", "", "", false)); err != nil {
				t.Fatalf("v1 create without SA: %v", err)
			}
			live := getLive()
			assertServiceAccountFields(t, live, "", "")
			assertLastAppliedSAFields(t, live, "", "", false)

			// 2. Upgrade adding serviceAccountName: "foo" (without deprecated serviceAccount).
			callerInput := makeWorkload(w.apiVersion, w.kind, "wl", "foo", "", false)
			if _, err := s.Apply(ctx, "chart", opts, callerInput); err != nil {
				t.Fatalf("v2 upgrade with serviceAccountName=foo: %v", err)
			}
			// Verify caller input was not mutated.
			ps, _ := nestedMapRef(callerInput.Object, podSpecPath(callerInput.GroupVersionKind().GroupKind()))
			if _, ok := ps["serviceAccount"]; ok {
				t.Errorf("caller input was mutated with serviceAccount field")
			}
			live = getLive()
			assertServiceAccountFields(t, live, "foo", "foo")
			assertLastAppliedSAFields(t, live, "foo", "foo", true)

			// 3. Re-apply same manifest -> patch must NOT touch spec/template.
			*patches = nil
			if _, err := s.Apply(ctx, "chart", opts, makeWorkload(w.apiVersion, w.kind, "wl", "foo", "", false)); err != nil {
				t.Fatalf("v3 no-op reapply: %v", err)
			}
			if len(*patches) != 1 {
				t.Fatalf("expected 1 patch on reapply, got %d", len(*patches))
			}
			var patchMap map[string]any
			if err := json.Unmarshal([]byte((*patches)[0]), &patchMap); err != nil {
				t.Fatal(err)
			}
			if _, hasSpec := patchMap["spec"]; hasSpec {
				t.Errorf("unexpected spec change in no-op patch: %s", (*patches)[0])
			}
			if _, hasTmpl := patchMap["template"]; hasTmpl {
				t.Errorf("unexpected template change in no-op patch: %s", (*patches)[0])
			}

			// 4. Rename SA foo -> bar (only serviceAccountName set).
			if _, err := s.Apply(ctx, "chart", opts, makeWorkload(w.apiVersion, w.kind, "wl", "bar", "", false)); err != nil {
				t.Fatalf("v4 rename SA foo->bar: %v", err)
			}
			live = getLive()
			assertServiceAccountFields(t, live, "bar", "bar")
			assertLastAppliedSAFields(t, live, "bar", "bar", true)

			// 5. Downgrade dropping serviceAccountName completely.
			if _, err := s.Apply(ctx, "chart", opts, makeWorkload(w.apiVersion, w.kind, "wl", "", "", false)); err != nil {
				t.Fatalf("v5 downgrade omitting SA: %v", err)
			}
			live = getLive()
			assertServiceAccountFields(t, live, "", "")
			assertLastAppliedSAFields(t, live, "", "", false)
		})
	}

	t.Run("old_synk_to_new_synk_downgrade_clears_both", func(t *testing.T) {
		// Simulate an object previously created by an OLD version of synk:
		// live has both serviceAccountName and serviceAccount set (via kube-apiserver
		// SetDefaults_PodSpec), while last-applied-configuration has ONLY serviceAccountName.
		legacy := makeWorkload("apps/v1", "Deployment", "wl", "foo", "", false)
		if err := setAppliedAnnotation(legacy); err != nil {
			t.Fatal(err)
		}
		applyServerPodSpecDefaults(legacy)
		assertServiceAccountFields(t, legacy, "foo", "foo")

		s, client, patches := newServerBackedSynk(t, false, legacy)
		depGVR := gvrs["deployments"]

		// Downgrade straight to manifest omitting serviceAccountName.
		if _, err := s.Apply(ctx, "chart", opts, makeWorkload("apps/v1", "Deployment", "wl", "", "", false)); err != nil {
			t.Fatalf("downgrade failed: %v", err)
		}
		live, err := client.Resource(depGVR).Namespace("app-ns").Get(ctx, "wl", metav1.GetOptions{})
		if err != nil {
			t.Fatal(err)
		}
		assertServiceAccountFields(t, live, "", "")
		assertLastAppliedSAFields(t, live, "", "", false)
		if len(*patches) != 1 || !strings.Contains((*patches)[0], `"serviceAccount":null`) || !strings.Contains((*patches)[0], `"serviceAccountName":null`) {
			t.Errorf("expected patch to clear both serviceAccount and serviceAccountName, got %v", *patches)
		}
	})

	t.Run("old_synk_to_new_synk_reapply_has_no_spec_diff_and_heals_annotation", func(t *testing.T) {
		legacy := makeWorkload("apps/v1", "Deployment", "wl", "foo", "", false)
		if err := setAppliedAnnotation(legacy); err != nil {
			t.Fatal(err)
		}
		applyServerPodSpecDefaults(legacy)

		s, client, patches := newServerBackedSynk(t, false, legacy)
		if _, err := s.Apply(ctx, "chart", opts, makeWorkload("apps/v1", "Deployment", "wl", "foo", "", false)); err != nil {
			t.Fatalf("reapply failed: %v", err)
		}
		var patchMap map[string]any
		if err := json.Unmarshal([]byte((*patches)[0]), &patchMap); err != nil {
			t.Fatal(err)
		}
		if _, hasSpec := patchMap["spec"]; hasSpec {
			t.Errorf("expected no spec patch when upgrading synk on unchanged Deployment, got: %s", (*patches)[0])
		}
		live, err := client.Resource(gvrs["deployments"]).Namespace("app-ns").Get(ctx, "wl", metav1.GetOptions{})
		if err != nil {
			t.Fatal(err)
		}
		assertLastAppliedSAFields(t, live, "foo", "foo", true)
	})

	t.Run("new_synk_to_old_synk_downgrade_clears_both", func(t *testing.T) {
		// Create with NEW synk using only serviceAccountName: "foo".
		s, client, _ := newServerBackedSynk(t, false)
		if _, err := s.Apply(ctx, "chart", opts, makeWorkload("apps/v1", "Deployment", "wl", "foo", "", false)); err != nil {
			t.Fatalf("create failed: %v", err)
		}
		live, err := client.Resource(gvrs["deployments"]).Namespace("app-ns").Get(ctx, "wl", metav1.GetOptions{})
		if err != nil {
			t.Fatal(err)
		}

		// Now simulate an UNPATCHED old synk / vanilla kubectl apply running
		// strategicpatch.CreateThreeWayMergePatch directly against live WITHOUT
		// any normalization helpers.
		originalRaw := getAppliedAnnotation(live)
		currentRaw, err := live.MarshalJSON()
		if err != nil {
			t.Fatal(err)
		}
		desired := makeWorkload("apps/v1", "Deployment", "wl", "", "", false)
		desiredRaw, err := desired.MarshalJSON()
		if err != nil {
			t.Fatal(err)
		}
		typed, err := scheme.Scheme.New(live.GroupVersionKind())
		if err != nil {
			t.Fatal(err)
		}
		patchMeta, err := strategicpatch.NewPatchMetaFromStruct(typed)
		if err != nil {
			t.Fatal(err)
		}
		patch, err := strategicpatch.CreateThreeWayMergePatch(originalRaw, desiredRaw, currentRaw, patchMeta, true)
		if err != nil {
			t.Fatal(err)
		}
		mergedBytes, err := strategicpatch.StrategicMergePatchUsingLookupPatchMeta(currentRaw, patch, patchMeta)
		if err != nil {
			t.Fatal(err)
		}
		var merged unstructured.Unstructured
		if err := merged.UnmarshalJSON(mergedBytes); err != nil {
			t.Fatal(err)
		}
		applyServerPodSpecDefaults(&merged)
		assertServiceAccountFields(t, &merged, "", "")
	})

	t.Run("deprecated_only_serviceAccount_downgrade_and_migration", func(t *testing.T) {
		// Previously applied with ONLY deprecated `serviceAccount: foo`.
		legacy := makeWorkload("apps/v1", "Deployment", "wl", "", "foo", false)
		if err := setAppliedAnnotation(legacy); err != nil {
			t.Fatal(err)
		}
		applyServerPodSpecDefaults(legacy)

		s, client, patches := newServerBackedSynk(t, false, legacy)

		// Migrating to `serviceAccountName: foo` should emit no spec patch.
		if _, err := s.Apply(ctx, "chart", opts, makeWorkload("apps/v1", "Deployment", "wl", "foo", "", false)); err != nil {
			t.Fatalf("migration failed: %v", err)
		}
		var patchMap map[string]any
		if err := json.Unmarshal([]byte((*patches)[0]), &patchMap); err != nil {
			t.Fatal(err)
		}
		if _, hasSpec := patchMap["spec"]; hasSpec {
			t.Errorf("unexpected spec patch on migration: %s", (*patches)[0])
		}

		live, err := client.Resource(gvrs["deployments"]).Namespace("app-ns").Get(ctx, "wl", metav1.GetOptions{})
		if err != nil {
			t.Fatal(err)
		}
		assertServiceAccountFields(t, live, "foo", "foo")
		assertLastAppliedSAFields(t, live, "foo", "foo", true)

		// Downgrading directly from deprecated-only last-applied clears both fields.
		legacy2 := makeWorkload("apps/v1", "Deployment", "wl2", "", "foo", false)
		if err := setAppliedAnnotation(legacy2); err != nil {
			t.Fatal(err)
		}
		applyServerPodSpecDefaults(legacy2)
		s2, client2, _ := newServerBackedSynk(t, false, legacy2)
		if _, err := s2.Apply(ctx, "chart", opts, makeWorkload("apps/v1", "Deployment", "wl2", "", "", false)); err != nil {
			t.Fatalf("downgrade failed: %v", err)
		}
		live2, err := client2.Resource(gvrs["deployments"]).Namespace("app-ns").Get(ctx, "wl2", metav1.GetOptions{})
		if err != nil {
			t.Fatal(err)
		}
		assertServiceAccountFields(t, live2, "", "")
	})

	t.Run("explicit_empty_string_and_null_clear_stuck_resource", func(t *testing.T) {
		// Simulate an already-stuck cluster resource: live has serviceAccount/serviceAccountName="foo",
		// but last-applied-configuration has NEITHER field (as left behind by an earlier old-synk downgrade).
		stuck := makeWorkload("apps/v1", "Deployment", "wl", "", "", false)
		if err := setAppliedAnnotation(stuck); err != nil {
			t.Fatal(err)
		}
		unstructured.SetNestedField(stuck.Object, "foo", "spec", "template", "spec", "serviceAccountName")
		unstructured.SetNestedField(stuck.Object, "foo", "spec", "template", "spec", "serviceAccount")

		s, client, _ := newServerBackedSynk(t, false, stuck)
		// Explicit `serviceAccountName: ""` clears both fields.
		if _, err := s.Apply(ctx, "chart", opts, makeWorkload("apps/v1", "Deployment", "wl", "", "", true)); err != nil {
			t.Fatalf("clear via empty string failed: %v", err)
		}
		live, err := client.Resource(gvrs["deployments"]).Namespace("app-ns").Get(ctx, "wl", metav1.GetOptions{})
		if err != nil {
			t.Fatal(err)
		}
		assertServiceAccountFields(t, live, "", "")

		// Same check with explicit `serviceAccountName: null`.
		stuck2 := makeWorkload("apps/v1", "Deployment", "wl2", "", "", false)
		if err := setAppliedAnnotation(stuck2); err != nil {
			t.Fatal(err)
		}
		unstructured.SetNestedField(stuck2.Object, "foo", "spec", "template", "spec", "serviceAccountName")
		unstructured.SetNestedField(stuck2.Object, "foo", "spec", "template", "spec", "serviceAccount")

		s2, client2, _ := newServerBackedSynk(t, false, stuck2)
		desiredNull := makeWorkload("apps/v1", "Deployment", "wl2", "", "", false)
		ps, _ := nestedMapRef(desiredNull.Object, []string{"spec", "template", "spec"})
		ps["serviceAccountName"] = nil
		if _, err := s2.Apply(ctx, "chart", opts, desiredNull); err != nil {
			t.Fatalf("clear via null failed: %v", err)
		}
		live2, err := client2.Resource(gvrs["deployments"]).Namespace("app-ns").Get(ctx, "wl2", metav1.GetOptions{})
		if err != nil {
			t.Fatal(err)
		}
		assertServiceAccountFields(t, live2, "", "")
	})

	t.Run("drop_both_workaround_back_to_serviceAccountName_only", func(t *testing.T) {
		s, client, patches := newServerBackedSynk(t, false)
		if _, err := s.Apply(ctx, "chart", opts, makeWorkload("apps/v1", "Deployment", "wl", "foo", "foo", false)); err != nil {
			t.Fatal(err)
		}
		*patches = nil
		if _, err := s.Apply(ctx, "chart", opts, makeWorkload("apps/v1", "Deployment", "wl", "foo", "", false)); err != nil {
			t.Fatal(err)
		}
		var patchMap map[string]any
		if err := json.Unmarshal([]byte((*patches)[0]), &patchMap); err != nil {
			t.Fatal(err)
		}
		if _, hasSpec := patchMap["spec"]; hasSpec {
			t.Errorf("unexpected spec patch dropping duplicate serviceAccount: %s", (*patches)[0])
		}
		live, err := client.Resource(gvrs["deployments"]).Namespace("app-ns").Get(ctx, "wl", metav1.GetOptions{})
		if err != nil {
			t.Fatal(err)
		}
		assertServiceAccountFields(t, live, "foo", "foo")
		assertLastAppliedSAFields(t, live, "foo", "foo", true)
	})

	t.Run("conflicting_fields_align_to_serviceAccountName", func(t *testing.T) {
		s, client, patches := newServerBackedSynk(t, false)
		if _, err := s.Apply(ctx, "chart", opts, makeWorkload("apps/v1", "Deployment", "wl", "new-sa", "old-sa", false)); err != nil {
			t.Fatal(err)
		}
		live, err := client.Resource(gvrs["deployments"]).Namespace("app-ns").Get(ctx, "wl", metav1.GetOptions{})
		if err != nil {
			t.Fatal(err)
		}
		assertServiceAccountFields(t, live, "new-sa", "new-sa")
		assertLastAppliedSAFields(t, live, "new-sa", "new-sa", true)

		*patches = nil
		if _, err := s.Apply(ctx, "chart", opts, makeWorkload("apps/v1", "Deployment", "wl", "new-sa", "old-sa", false)); err != nil {
			t.Fatal(err)
		}
		var patchMap map[string]any
		if err := json.Unmarshal([]byte((*patches)[0]), &patchMap); err != nil {
			t.Fatal(err)
		}
		if _, hasSpec := patchMap["spec"]; hasSpec {
			t.Errorf("expected no spurious spec patch on re-apply, got: %s", (*patches)[0])
		}
	})

	t.Run("pod_default_serviceAccount_preserved_when_unspecified", func(t *testing.T) {
		s, client, patches := newServerBackedSynk(t, false)
		if _, err := s.Apply(ctx, "chart", opts, makeWorkload("v1", "Pod", "p", "", "", false)); err != nil {
			t.Fatal(err)
		}
		*patches = nil
		if _, err := s.Apply(ctx, "chart", opts, makeWorkload("v1", "Pod", "p", "", "", false)); err != nil {
			t.Fatal(err)
		}
		var patchMap map[string]any
		if err := json.Unmarshal([]byte((*patches)[0]), &patchMap); err != nil {
			t.Fatal(err)
		}
		if _, hasSpec := patchMap["spec"]; hasSpec {
			t.Errorf("Pod re-apply must not patch server-defaulted spec.serviceAccountName, got: %s", (*patches)[0])
		}
		pod, err := client.Resource(schema.GroupVersionResource{Version: "v1", Resource: "pods"}).Namespace("app-ns").Get(ctx, "p", metav1.GetOptions{})
		if err != nil {
			t.Fatal(err)
		}
		assertServiceAccountFields(t, pod, "default", "default")
	})

	t.Run("job_downgrade_triggers_replace_when_template_immutable", func(t *testing.T) {
		legacyJob := makeWorkload("batch/v1", "Job", "j", "foo", "", false)
		if err := setAppliedAnnotation(legacyJob); err != nil {
			t.Fatal(err)
		}
		applyServerPodSpecDefaults(legacyJob)

		s, client, _ := newServerBackedSynk(t, true, legacyJob)

		// First re-apply unchanged Job: must NOT fail or replace.
		if _, err := s.Apply(ctx, "chart", opts, makeWorkload("batch/v1", "Job", "j", "foo", "", false)); err != nil {
			t.Fatalf("unchanged Job reapply failed: %v", err)
		}
		jobGVR := schema.GroupVersionResource{Group: "batch", Version: "v1", Resource: "jobs"}
		live, err := client.Resource(jobGVR).Namespace("app-ns").Get(ctx, "j", metav1.GetOptions{})
		if err != nil {
			t.Fatal(err)
		}
		assertServiceAccountFields(t, live, "foo", "foo")

		// Now downgrade dropping serviceAccountName: triggers immutable error -> replace!
		if _, err := s.Apply(ctx, "chart", opts, makeWorkload("batch/v1", "Job", "j", "", "", false)); err != nil {
			t.Fatalf("Job downgrade replace failed: %v", err)
		}
		live, err = client.Resource(jobGVR).Namespace("app-ns").Get(ctx, "j", metav1.GetOptions{})
		if err != nil {
			t.Fatal(err)
		}
		assertServiceAccountFields(t, live, "", "")
		assertLastAppliedSAFields(t, live, "", "", false)
	})

	t.Run("custom_resource_untouched", func(t *testing.T) {
		f := newFixture(t)
		rollout := newUnstructured("apps.cloudrobotics.com/v1alpha1", "AppRollout", "app-ns", "r1")
		rollout.Object["spec"] = map[string]any{
			"template": map[string]any{
				"spec": map[string]any{
					"serviceAccountName": "foo",
				},
			},
		}
		set := &apps.ResourceSet{}
		set.Name = "test.v1"
		set.UID = "deadbeef"
		if _, err := f.newSynk().applyAll(ctx, set, opts, rollout.DeepCopy()); err != nil {
			t.Fatal(err)
		}
		for _, a := range f.fake.Actions() {
			if ca, ok := a.(k8stest.CreateAction); ok && ca.GetResource().Resource == "approllouts" {
				u := ca.GetObject().(*unstructured.Unstructured)
				_, found, _ := unstructured.NestedString(u.Object, "spec", "template", "spec", "serviceAccount")
				if found {
					t.Errorf("custom resource must not have serviceAccount injected")
				}
			}
		}
	})
}
