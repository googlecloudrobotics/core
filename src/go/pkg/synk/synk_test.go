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
	"testing"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"k8s.io/apimachinery/pkg/api/meta/testrestmapper"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/client-go/discovery"
	dynamicfake "k8s.io/client-go/dynamic/fake"
	"k8s.io/client-go/kubernetes/scheme"
	k8stest "k8s.io/client-go/testing"
	"sigs.k8s.io/yaml"
)

type fakeCachedDiscoveryClient struct {
	discovery.CachedDiscoveryInterface
}

func (d *fakeCachedDiscoveryClient) Invalidate() {}

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
	var (
		client = dynamicfake.NewSimpleDynamicClient(scheme.Scheme, f.objects...)
		s      = New(client, &fakeCachedDiscoveryClient{})
	)
	s.mapper = testrestmapper.TestOnlyStaticRESTMapper(scheme.Scheme)
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
	want := unmarshalUnstructured(t, `
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

// Hardcode some GVR mappings for easy use in tests. The only other way is
// setting up a full RestMapper.
var gvrs = map[string]schema.GroupVersionResource{
	"configmaps":  {Version: "v1", Resource: "configmaps"},
	"deployments": {Group: "apps", Version: "v1", Resource: "deployments"},
}

func TestSynk_applyAll(t *testing.T) {
	cm1 := newUnstructured("v1", "ConfigMap", "foo1", "cm1")
	cm2 := newUnstructured("v1", "ConfigMap", "foo1", "cm2")
	dp1 := newUnstructured("apps/v1", "Deployment", "foo2", "dp1")

	f := newFixture(t)
	// cm1 already exists beforehand, so we expect an update.
	f.addObjects(cm1)

	synk := f.newSynk()

	err := synk.applyAll(&apps.ResourceSet{}, &ApplyOptions{name: "test"}, cm1, cm2, dp1)
	if err != nil {
		t.Fatal(err)
	}
	f.expectActions(
		k8stest.NewUpdateAction(gvrs["configmaps"], "foo1", cm1),
		k8stest.NewCreateAction(gvrs["configmaps"], "foo1", cm2),
		k8stest.NewCreateAction(gvrs["deployments"], "foo2", dp1),
	)
	f.verifyWriteActions()
}

func newUnstructured(apiVersion, kind, namespace, name string) *unstructured.Unstructured {
	var u unstructured.Unstructured
	u.SetAPIVersion(apiVersion)
	u.SetKind(kind)
	u.SetNamespace(namespace)
	u.SetName(name)
	return &u
}

func unmarshalUnstructured(t *testing.T, s string) *unstructured.Unstructured {
	var u unstructured.Unstructured
	if err := yaml.Unmarshal([]byte(s), &u.Object); err != nil {
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
	default:
		return fmt.Sprintf("<UNKNOWN ACTION %T>", a)
	}
}
