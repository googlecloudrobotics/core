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
	"fmt"
	"reflect"
	"strings"
	"testing"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"github.com/pkg/errors"
	corev1 "k8s.io/api/core/v1"
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

func TestSynk_initialize(t *testing.T) {
	s := newFixture(t).newSynk()

	_, _, err := s.initialize(&ApplyOptions{name: "test"},
		newUnstructured("v1", "Pod", "ns2", "pod1"),
		newUnstructured("apps/v1", "Deployment", "ns1", "deploy1"),
		newUnstructured("v1", "Pod", "ns1", "pod1"),
	)
	if err != nil {
		t.Fatal(err)
	}
	got, err := s.client.Resource(resourceSetGVR).Get("test.v1", metav1.GetOptions{})
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
	f := newFixture(t)
	s := f.newSynk()

	rs := &apps.ResourceSet{
		ObjectMeta: metav1.ObjectMeta{Name: "set1"},
	}
	if err := s.createResourceSet(rs); err != nil {
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
	err := s.updateResourceSetStatus(rs, results)
	if err != nil {
		t.Fatal(err)
	}
	got, err := s.client.Resource(resourceSetGVR).Get("set1", metav1.GetOptions{})
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

func TestSynk_applyAll(t *testing.T) {
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

	// Support for a standard merge-patch was only added to the client-go testing
	// mock as of kubernetes-1.14.0, which we cannot upgrade to yet.
	// Thus we cannot valid the standard merge patch type for CRDs yet.
	rollout := newUnstructured("apps.cloudrobotics.com/v1alpha1", "AppRollout", "foo1", "rollout1")
	deploy := newUnstructured("apps/v1", "Deployment", "foo2", "dp1")

	unmarshalYAML(t, &cmUpdate, `
apiVersion: v1
kind: ConfigMap
metadata:
  namespace: foo1
  name: cm1
data:
  foo2: baz2
  foo3: bar3`)

	set := &apps.ResourceSet{}
	set.Name = "test.v1"
	set.UID = "deadbeef"

	results, err := f.newSynk().applyAll(set, &ApplyOptions{name: "test"},
		toUnstructured(t, &cmUpdate).DeepCopy(),
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

	cmPatch := []byte(`{"data":{"foo2":"baz2","foo3":"bar3"},"metadata":{"annotations":{"kubectl.kubernetes.io/last-applied-configuration":"{\"apiVersion\":\"v1\",\"data\":{\"foo2\":\"baz2\",\"foo3\":\"bar3\"},\"kind\":\"ConfigMap\",\"metadata\":{\"annotations\":{},\"creationTimestamp\":null,\"name\":\"cm1\",\"namespace\":\"foo1\",\"ownerReferences\":[{\"apiVersion\":\"apps.cloudrobotics.com/v1alpha1\",\"blockOwnerDeletion\":true,\"kind\":\"ResourceSet\",\"name\":\"test.v1\",\"uid\":\"deadbeef\"}]}}\n"},"ownerReferences":[{"apiVersion":"apps.cloudrobotics.com/v1alpha1","blockOwnerDeletion":true,"kind":"ResourceSet","name":"test.v1","uid":"deadbeef"}]}}`)

	f.expectActions(
		k8stest.NewPatchAction(gvrs["configmaps"], "foo1", "cm1", types.StrategicMergePatchType, cmPatch),
		k8stest.NewCreateAction(gvrs["approllouts"], "foo1", rollout),
		k8stest.NewCreateAction(gvrs["deployments"], "foo2", deploy),
	)
	f.verifyWriteActions()
}

func TestSynk_deleteResourceSets(t *testing.T) {
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

	err := synk.deleteResourceSets("test", 7)
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
apiVersion: apiextensions.k8s.io/v1beta1
kind: CustomResourceDefition
metadata:
  name: examples.example.org
spec:
  group: example.org
  version: v1
  names:
    kind: Example
  scope: Namespaced`)

	if err := s.populateNamespaces(
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
