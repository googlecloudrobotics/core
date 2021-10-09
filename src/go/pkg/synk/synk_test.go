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
	"context"
	"fmt"
	"reflect"
	"strings"
	"testing"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"github.com/pkg/errors"
	corev1 "k8s.io/api/core/v1"
	k8serrors "k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/api/meta/testrestmapper"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/apimachinery/pkg/types"
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
}

func (d *fakeCachedDiscoveryClient) Invalidate() {}

func (d *fakeCachedDiscoveryClient) ServerResources() ([]*metav1.APIResourceList, error) {
	return []*metav1.APIResourceList{
		{
			GroupVersion: "v1",
			APIResources: []metav1.APIResource{
				{Kind: "Pod", Namespaced: true},
				{Kind: "Namespace", Namespaced: false},
			},
		},
	}, nil
}

type fixture struct {
	*testing.T
	fake *k8stest.Fake

	// Starting state the respective client will report.
	objects []runtime.Object
	// Actions we want to see called against the respective client.
	actions []k8stest.Action
}

func newFixture(t *testing.T) *fixture {
	return &fixture{T: t}
}

func (f *fixture) newSynk() *Synk {
	sc := runtime.NewScheme()
	scheme.AddToScheme(sc)
	apps.AddToScheme(sc) // For tests with CRDs.
	var (
		client = dynamicfake.NewSimpleDynamicClient(sc, f.objects...)
		s      = New(client, &fakeCachedDiscoveryClient{})
	)
	s.mapper = testrestmapper.TestOnlyStaticRESTMapper(sc)
	s.resetMapper = func() {}
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
		err  error
		want bool
	}{
		{
			errors.New("generic error"),
			false,
		},
		{
			transientErr{errors.New("transientErr struct")},
			true,
		},
		{
			&transientErr{errors.New("transientErr pointer")},
			true,
		},
		{
			k8serrors.NewUnauthorized("unauthorized"),
			false,
		},
		{
			k8serrors.NewResourceExpired("gone"),
			true,
		},
		{
			k8serrors.NewForbidden(schema.GroupResource{
				Group:    "apps",
				Resource: "deployments",
			}, "my-deployment", errors.New("unable to create new content in namespace app-test-chart because it is being terminated")),
			true,
		},
	}

	for _, tc := range tests {
		t.Run(tc.err.Error(), func(t *testing.T) {
			if got := IsTransientErr(tc.err); got != tc.want {
				t.Errorf("IsTransientErr(%v)=%v, want %v", tc.err, got, tc.want)
			}
		})
	}
}

// TODO(rodrigoq): test Apply() directly rather than the private methods
func TestSynk_initialize(t *testing.T) {
	ctx := context.Background()
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
	ctx := context.Background()
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
	results, err := f.newSynk().applyAll(context.Background(), set, &ApplyOptions{name: "test"},
		cm.DeepCopy(),
	)
	if err != nil {
		t.Error(err)
		for _, res := range results {
			t.Log(res)
		}
		return
	}

	_true := true
	ownerRef := metav1.OwnerReference{
		APIVersion:         "apps.cloudrobotics.com/v1alpha1",
		Kind:               "ResourceSet",
		Name:               set.Name,
		UID:                set.UID,
		BlockOwnerDeletion: &_true,
	}
	cm.SetOwnerReferences([]metav1.OwnerReference{ownerRef})

	setAppliedAnnotation(cm)

	f.expectActions(
		k8stest.NewUpdateAction(gvrs["configmaps"], "foo1", cm),
	)
	f.verifyWriteActions()
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

	results, err := f.newSynk().applyAll(context.Background(), set, &ApplyOptions{name: "test"},
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

	_true := true
	ownerRef := metav1.OwnerReference{
		APIVersion:         "apps.cloudrobotics.com/v1alpha1",
		Kind:               "ResourceSet",
		Name:               set.Name,
		UID:                set.UID,
		BlockOwnerDeletion: &_true,
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
	_true := true
	ownerRef := metav1.OwnerReference{
		APIVersion:         "apps.cloudrobotics.com/v1alpha1",
		Kind:               "ResourceSet",
		Name:               set.Name,
		UID:                set.UID,
		BlockOwnerDeletion: &_true,
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

			_, err := s.applyAll(context.Background(), set, &ApplyOptions{name: "test"},
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

func TestSynk_skipsTestResources(t *testing.T) {
	ctx := context.Background()
	s := newFixture(t).newSynk()

	testPod := newUnstructured("v1", "Pod", "ns", "pod2")
	testPod.SetAnnotations(map[string]string{"helm.sh/hook": "test-success"})

	_, _, err := s.initialize(context.Background(), &ApplyOptions{name: "test"},
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
	ctx := context.Background()
	f := newFixture(t)
	f.addObjects(
		newUnstructured("apps.cloudrobotics.com/v1alpha1", "ResourceSet", "", "test.v2"),
		newUnstructured("apps.cloudrobotics.com/v1alpha1", "ResourceSet", "", "bad_name"),
		newUnstructured("apps.cloudrobotics.com/v1alpha1", "ResourceSet", "", "other.v3"),
		newUnstructured("apps.cloudrobotics.com/v1alpha1", "ResourceSet", "", "test.v4"),
		newUnstructured("apps.cloudrobotics.com/v1alpha1", "ResourceSet", "", "test.v7"),
		newUnstructured("apps.cloudrobotics.com/v1alpha1", "ResourceSet", "", "test.v8"),
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
		context.Background(),
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
