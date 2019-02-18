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

package main

import (
	"fmt"
	"reflect"
	"testing"
	"time"

	crdtypes "k8s.io/apiextensions-apiserver/pkg/apis/apiextensions/v1beta1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/runtime/schema"
	k8sfake "k8s.io/client-go/dynamic/fake"
	k8stest "k8s.io/client-go/testing"
	"k8s.io/client-go/tools/cache"
	"k8s.io/client-go/util/workqueue"
)

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

type fixture struct {
	*testing.T

	local  *k8sfake.FakeDynamicClient
	remote *k8sfake.FakeDynamicClient

	// Starting state the respective client will report.
	remoteObjects []runtime.Object
	localObjects  []runtime.Object
	// Actions we want to see called against the respective client.
	remoteActions []k8stest.Action
	localActions  []k8stest.Action
}

func newFixture(t *testing.T) *fixture {
	return &fixture{T: t}
}

func (f *fixture) newCRSyncer(crd crdtypes.CustomResourceDefinition, robotName string) (*crSyncer, schema.GroupVersionResource) {
	gvk := schema.GroupVersionKind{
		Group:   crd.Spec.Group,
		Version: crd.Spec.Version,
		Kind:    crd.Spec.Names.Kind,
	}
	s := runtime.NewScheme()
	s.AddKnownTypeWithName(gvk, &unstructured.Unstructured{})

	f.local = k8sfake.NewSimpleDynamicClient(s, f.localObjects...)
	f.remote = k8sfake.NewSimpleDynamicClient(s, f.remoteObjects...)

	crs, err := newCRSyncer(crd, f.local, f.remote, robotName)
	if err != nil {
		f.Fatal(err)
	}
	return crs, schema.GroupVersionResource{
		Group:    crd.Spec.Group,
		Version:  crd.Spec.Version,
		Resource: crd.Spec.Names.Plural,
	}
}

func (f *fixture) addLocalObjects(objs ...runtime.Object) {
	f.localObjects = append(f.localObjects, objs...)
}

func (f *fixture) addRemoteObjects(objs ...runtime.Object) {
	f.remoteObjects = append(f.remoteObjects, objs...)
}

func (f *fixture) expectLocalActions(as ...k8stest.Action) {
	f.localActions = append(f.localActions, as...)
}

func (f *fixture) expectRemoteActions(as ...k8stest.Action) {
	f.remoteActions = append(f.remoteActions, as...)
}

func (f *fixture) verifyWriteActions() {
	var (
		localWrites  = filterReadActions(f.local.Actions())
		remoteWrites = filterReadActions(f.remote.Actions())
	)
	if !reflect.DeepEqual(localWrites, f.localActions) {
		f.Errorf("local writes did not match")
		f.Logf("received:")
		for i, a := range localWrites {
			f.Logf("%d: %s", i, sprintAction(a))
		}
		f.Logf("expected:")
		for i, a := range f.localActions {
			f.Logf("%d: %s", i, sprintAction(a))
		}
	}
	if !reflect.DeepEqual(remoteWrites, f.remoteActions) {
		f.Errorf("remote writes did not match")
		f.Logf("received:")
		for i, a := range remoteWrites {
			f.Logf("%d: %s", i, sprintAction(a))
		}
		f.Logf("expected:")
		for i, a := range f.remoteActions {
			f.Logf("%d: %s", i, sprintAction(a))
		}
	}
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

// testCRD returns a basic resource definition we use for custom testing.
// It may be altered for specific tests.
// By default it has set the spec-source annotation to "cloud".
func testCRD() crdtypes.CustomResourceDefinition {
	return crdtypes.CustomResourceDefinition{
		ObjectMeta: metav1.ObjectMeta{
			Name:   "goals.crds.example.com",
			Labels: map[string]string{},
			Annotations: map[string]string{
				annotationSpecSource: "cloud",
			},
		},
		Spec: crdtypes.CustomResourceDefinitionSpec{
			Group:   "crds.example.com",
			Version: "v1beta1",
			Names: crdtypes.CustomResourceDefinitionNames{
				Kind:     "Goal",
				Singular: "goal",
				Plural:   "goals",
			},
		},
	}
}

// newTestCR creates a new custom resource that matches the definition of testCRD().
func newTestCR(name string, spec, status interface{}) *unstructured.Unstructured {
	o := &unstructured.Unstructured{}

	o.SetKind("Goal")
	o.SetAPIVersion("crds.example.com/v1beta1")
	o.SetNamespace(metav1.NamespaceDefault)
	o.SetName(name)

	o.Object["spec"] = spec
	o.Object["status"] = status

	return o
}

func TestSyncUpstream_createSpec(t *testing.T) {
	crd := testCRD()
	f := newFixture(t)

	// When an upstream resource is seen for the first time, it should be
	// created in the downstream cluster including its current status.
	// The finalizer must be added upstream and downstream.
	tcrRemote := newTestCR("resource1", "spec1", "status1")
	f.addRemoteObjects(tcrRemote)

	crs, gvr := f.newCRSyncer(crd, "cluster1")
	defer crs.stop()

	crs.startInformers()
	if err := crs.syncUpstream("default/resource1"); err != nil {
		t.Fatal(err)
	}
	var (
		finalizer    = finalizerFor("robot-cluster1")
		tcrLocalNew  = newTestCR("resource1", "spec1", "status1")
		tcrRemoteNew = newTestCR("resource1", "spec1", "status1")
	)
	tcrLocalNew.SetFinalizers([]string{finalizer})
	tcrRemoteNew.SetFinalizers([]string{finalizer})

	f.expectRemoteActions(k8stest.NewUpdateAction(gvr, "default", tcrRemoteNew))
	f.expectLocalActions(k8stest.NewCreateAction(gvr, "default", tcrLocalNew))
	f.verifyWriteActions()
}

func TestSyncUpstream_updateSpec(t *testing.T) {
	crd := testCRD()
	f := newFixture(t)

	// On upstream update, the spec in the downstream cluster should be adjusted
	// and the finalizer be set in both clusters.
	var (
		tcrLocal  = newTestCR("resource1", "spec1", "status2")
		tcrRemote = newTestCR("resource1", "spec2", "status1")
	)
	f.addLocalObjects(tcrLocal)
	f.addRemoteObjects(tcrRemote)

	crs, gvr := f.newCRSyncer(crd, "cluster1")
	defer crs.stop()

	crs.startInformers()
	if err := crs.syncUpstream("default/resource1"); err != nil {
		t.Fatal(err)
	}
	var (
		finalizer    = finalizerFor("robot-cluster1")
		tcrLocalNew  = newTestCR("resource1", "spec2", "status2")
		tcrRemoteNew = newTestCR("resource1", "spec2", "status1")
	)
	tcrLocalNew.SetFinalizers([]string{finalizer})
	tcrRemoteNew.SetFinalizers([]string{finalizer})

	f.expectRemoteActions(k8stest.NewUpdateAction(gvr, "default", tcrRemoteNew))
	f.expectLocalActions(k8stest.NewUpdateAction(gvr, "default", tcrLocalNew))
	f.verifyWriteActions()
}

func TestSyncUpstream_propagateDelete(t *testing.T) {
	crd := testCRD()
	f := newFixture(t)

	var (
		now       = metav1.Now()
		finalizer = finalizerFor("robot-cluster1")
		tcrLocal  = newTestCR("resource1", "spec1", "status1")
		tcrRemote = newTestCR("resource1", "spec1", "status1")
	)
	tcrRemote.SetDeletionTimestamp(&now)
	tcrRemote.SetFinalizers([]string{finalizer})
	tcrLocal.SetFinalizers([]string{finalizer})

	f.addLocalObjects(tcrLocal)
	f.addRemoteObjects(tcrRemote)

	crs, gvr := f.newCRSyncer(crd, "cluster1")
	defer crs.stop()

	crs.startInformers()
	if err := crs.syncUpstream("default/resource1"); err != nil {
		t.Fatal(err)
	}

	tcrLocalNew := newTestCR("resource1", "spec1", "status1")
	tcrLocalNew.SetFinalizers([]string{finalizer})

	f.expectLocalActions(
		k8stest.NewUpdateAction(gvr, "default", tcrLocalNew),
		k8stest.NewDeleteAction(gvr, "default", "resource1"),
	)
	f.verifyWriteActions()
}

func TestSyncDownstream_deleteOrphan(t *testing.T) {
	crd := testCRD()
	f := newFixture(t)

	// We have a local resource that has no matching resource in the upstream cluster.
	// Trying to sync it again should delete the local copy and remove the finalizer.
	tcrLocal := newTestCR("resource1", "spec1", "status1")
	tcrLocal.SetFinalizers([]string{finalizerFor("robot-cluster1")})

	f.addLocalObjects(tcrLocal)

	crs, gvr := f.newCRSyncer(crd, "cluster1")
	defer crs.stop()

	crs.startInformers()
	if err := crs.syncDownstream("default/resource1"); err != nil {
		t.Fatal(err)
	}

	tcrRemoteNew := newTestCR("resource1", "spec1", "status1")

	f.expectLocalActions(
		k8stest.NewUpdateAction(gvr, "default", tcrRemoteNew),
		k8stest.NewDeleteAction(gvr, "default", "resource1"),
	)
	f.verifyWriteActions()
}

func TestSyncDownstream_statusFull(t *testing.T) {
	crd := testCRD()
	f := newFixture(t)

	var (
		tcrLocal  = newTestCR("resource1", "spec1", "status2")
		tcrRemote = newTestCR("resource1", "spec1", "status1")
	)
	tcrLocal.SetResourceVersion("123")

	f.addLocalObjects(tcrLocal)
	f.addRemoteObjects(tcrRemote)

	crs, gvr := f.newCRSyncer(crd, "")
	defer crs.stop()

	crs.startInformers()
	if err := crs.syncDownstream("default/resource1"); err != nil {
		t.Fatal(err)
	}

	tcrRemoteNew := newTestCR("resource1", "spec1", "status2")
	tcrRemoteNew.SetAnnotations(map[string]string{
		annotationResourceVersion: "123",
	})

	f.expectRemoteActions(k8stest.NewUpdateAction(gvr, "default", tcrRemoteNew))
	f.verifyWriteActions()
}

func TestSyncDownstream_statusSubtree(t *testing.T) {
	crd := testCRD()
	f := newFixture(t)

	var (
		tcrLocal = newTestCR("resource1", "spec1", map[string]interface{}{
			"cloud": "cloud_1",
			"robot": "robot_2",
		})
		tcrRemote = newTestCR("resource1", "spec1", map[string]interface{}{
			"cloud": "cloud_2",
			"robot": "robot_1",
		})
	)
	tcrLocal.SetResourceVersion("123")

	f.addLocalObjects(tcrLocal)
	f.addRemoteObjects(tcrRemote)

	crs, gvr := f.newCRSyncer(crd, "")
	defer crs.stop()

	crs.subtree = "robot"
	crs.startInformers()
	if err := crs.syncDownstream("default/resource1"); err != nil {
		t.Fatal(err)
	}

	tcrRemoteNew := newTestCR("resource1", "spec1", map[string]interface{}{
		"cloud": "cloud_2",
		"robot": "robot_2",
	})
	tcrRemoteNew.SetAnnotations(map[string]string{
		annotationResourceVersion: "123",
	})

	f.expectRemoteActions(k8stest.NewUpdateAction(gvr, "default", tcrRemoteNew))
	f.verifyWriteActions()
}

func TestSyncDownstream_removeFinalizers(t *testing.T) {
	crd := testCRD()
	f := newFixture(t)

	// If a deletion timestamp is set on the local resource, the finalizer
	// should be removed from upstream and downstream resource.
	var (
		now       = metav1.Now()
		finalizer = finalizerFor("robot-cluster1")
		tcrLocal  = newTestCR("resource1", "spec1", "status1")
		tcrRemote = newTestCR("resource1", "spec1", "status1")
	)
	tcrLocal.SetDeletionTimestamp(&now)
	tcrLocal.SetResourceVersion("123")
	tcrLocal.SetFinalizers([]string{finalizer})
	tcrRemote.SetFinalizers([]string{finalizer})

	f.addLocalObjects(tcrLocal)
	f.addRemoteObjects(tcrRemote)

	crs, gvr := f.newCRSyncer(crd, "cluster1")
	defer crs.stop()

	crs.startInformers()
	if err := crs.syncDownstream("default/resource1"); err != nil {
		t.Fatal(err)
	}

	var (
		tcrLocalNew   = newTestCR("resource1", "spec1", "status1")
		tcrRemoteNew1 = newTestCR("resource1", "spec1", "status1")
		tcrRemoteNew2 = newTestCR("resource1", "spec1", "status1")
	)
	tcrLocalNew.SetResourceVersion("123")
	tcrLocalNew.SetDeletionTimestamp(&now)
	tcrRemoteNew1.SetFinalizers([]string{finalizer})
	tcrRemoteNew1.SetAnnotations(map[string]string{
		annotationResourceVersion: "123",
	})
	tcrRemoteNew2.SetAnnotations(map[string]string{
		annotationResourceVersion: "123",
	})

	f.expectLocalActions(
		k8stest.NewUpdateAction(gvr, "default", tcrLocalNew),
	)
	f.expectRemoteActions(
		k8stest.NewUpdateAction(gvr, "default", tcrRemoteNew1),
		k8stest.NewUpdateAction(gvr, "default", tcrRemoteNew2),
	)
	f.verifyWriteActions()
}

func TestCRSyncer_populateWorkqueue(t *testing.T) {
	crd := testCRD()
	f := newFixture(t)

	cr1 := newTestCR("cr1", "spec1", "status1")
	f.addRemoteObjects(cr1)

	crs, _ := f.newCRSyncer(crd, "")
	defer crs.stop()
	crs.startInformers()

	// Workqueue exposes no interface to select{} over, so we call Get()
	// in a goroutine to surface deadlocks properly.
	channel := make(chan interface{}, 1)
	go func() {
		key, quit := crs.upstreamQueue.Get()
		if quit {
			t.Errorf("unexpected quit")
			return
		}
		item, exists, err := crs.upstreamInf.GetIndexer().GetByKey(key.(string))
		if err != nil {
			t.Errorf("unexpected lookup error for key %s: %s", key, err)
		}
		if !exists {
			t.Errorf("item for key %s does not exist", key)
		} else {
			channel <- item
		}
	}()

	select {
	case obj := <-channel:
		if got := obj.(*unstructured.Unstructured); !reflect.DeepEqual(got, cr1) {
			t.Errorf("unexpected object; want %v; got %v", cr1, got)
		}
	case <-time.After(5 * time.Second):
		t.Errorf("Received no watch event; wanted %v", cr1)
	}
}

func TestCRSyncer_populateWorkqueueWithFilter(t *testing.T) {
	crd := testCRD()
	crd.ObjectMeta.Annotations[annotationFilterByRobotName] = "true"

	f := newFixture(t)

	// Create three CRs of which only one matches the robot the CR syncer
	// is running on.
	crCorrectRobot := newTestCR("cr1", "spec1", "status1")
	crWrongRobot := newTestCR("cr2", "spec2", "status2")
	crNoRobot := newTestCR("cr3", "spec3", "status3")

	crCorrectRobot.SetLabels(map[string]string{labelRobotName: "robot-1"})
	crWrongRobot.SetLabels(map[string]string{labelRobotName: "robot-2"})

	f.addRemoteObjects(crCorrectRobot, crWrongRobot, crNoRobot)

	crs, _ := f.newCRSyncer(crd, "robot-1")
	defer crs.stop()
	crs.startInformers()

	channel := channelFromQueue(t, crs.upstreamQueue, crs.upstreamInf)
	select {
	case got := <-channel:
		if !reflect.DeepEqual(got, crCorrectRobot) {
			t.Errorf("unexpected object; want %v; got %v", crCorrectRobot, got)
		}
	case <-time.After(5 * time.Second):
		t.Errorf("Received no watch event; wanted %v", crCorrectRobot)
	}
	// No other items should come through.
	select {
	case item := <-channel:
		t.Errorf("Unexpected update: %v", item)
	case <-time.After(3 * time.Second):
	}
}

func channelFromQueue(t *testing.T, queue workqueue.Interface, inf cache.SharedIndexInformer) <-chan *unstructured.Unstructured {
	ch := make(chan *unstructured.Unstructured, 1)
	go func() {
		defer close(ch)
		for {
			key, quit := queue.Get()
			if quit {
				t.Errorf("unexpected quit")
				return
			}
			item, exists, err := inf.GetIndexer().GetByKey(key.(string))
			if err != nil {
				t.Errorf("unexpected lookup error for key %s: %s", key, err)
			}
			if !exists {
				t.Errorf("item for key %s does not exist", key)
			} else {
				ch <- item.(*unstructured.Unstructured)
			}
		}
	}()
	return ch

}
