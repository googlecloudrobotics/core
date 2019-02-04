// Copyright 2019 The Google Cloud Robotics Authors
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
	"reflect"
	"testing"
	"time"

	"github.com/h2non/gock"
	crdtypes "k8s.io/apiextensions-apiserver/pkg/apis/apiextensions/v1beta1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/client-go/dynamic"
	k8sfake "k8s.io/client-go/dynamic/fake"
	"k8s.io/client-go/rest"
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
	if len(localWrites) != len(f.localActions) {
		f.Errorf("expected %d local writes but got %d", len(localWrites), len(f.localActions))
	}
	if !reflect.DeepEqual(localWrites, f.localActions) {
		f.Errorf("received local writes: %v", localWrites)
		f.Errorf("expected local writes: %v", f.localActions)
	}
	if len(remoteWrites) != len(f.remoteActions) {
		f.Errorf("expected %d remote writes but got %d", len(remoteWrites), len(f.remoteActions))
	}
	if !reflect.DeepEqual(remoteWrites, f.remoteActions) {
		f.Errorf("received remote writes: %v", remoteWrites)
		f.Errorf("expected remote writes: %v", f.remoteActions)
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

func TestCRSyncer_syncDeletes(t *testing.T) {
	crd := testCRD()
	f := newFixture(t)

	var (
		// Two local-only resources only one of which is owned by the
		// remote and should be deleted.
		tcrLocal1 = newTestCR("local1", "spec1", "status1")
		tcrLocal2 = newTestCR("local2", "spec1", "status1")
		tcrRemote = newTestCR("remote", "spec1", "status1")
		tcrShared = newTestCR("shared", "spec1", "status1")
	)
	setAnnotation(tcrLocal1, annotationOwnedByUpstream, "true")

	f.addLocalObjects(tcrLocal1, tcrLocal2, tcrShared)
	f.addRemoteObjects(tcrRemote, tcrShared)

	crs, gvr := f.newCRSyncer(crd, "")
	defer crs.stop()

	// After reconciliation we expect that the resource that only lived
	// locally has been deleted.
	// The other ones remain unchanged. The remote-only resource has not
	// been copied to the local cluster since that's not part of the
	// deletion reconciliation.
	crs.startInformers()
	crs.syncDeletes()

	f.expectLocalActions(k8stest.NewDeleteAction(gvr, metav1.NamespaceDefault, "local1"))

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

func TestCRSyncer_copySpec_handleDelete(t *testing.T) {
	f := newFixture(t)
	f.addLocalObjects(newTestCR("cr1", "spec1", "status1"))

	crs, gvr := f.newCRSyncer(testCRD(), "")
	defer crs.stop()
	crs.startInformers()

	crs.copySpec(metav1.NamespaceDefault + "/cr1")

	f.expectLocalActions(k8stest.NewDeleteAction(gvr, metav1.NamespaceDefault, "cr1"))
	f.verifyWriteActions()
}

func TestCRSyncer_copySpec_handleDeleteErrors(t *testing.T) {
	defer gock.Off()
	// First we return a 500 error, which should be handed down.
	gock.New("http://remote-server/").
		Delete("/apis/crds.example.com/v1beta1/namespaces/default/goals/foo").
		Reply(500)
	// The 404 error should be ignored to handle double deletions.
	gock.New("http://remote-server/").
		Delete("/apis/crds.example.com/v1beta1/namespaces/default/goals/foo").
		Reply(404)

	f := newFixture(t)
	crs, gvr := f.newCRSyncer(testCRD(), "")

	client, err := dynamic.NewForConfig(&rest.Config{
		Host: "remote-server",
	})
	if err != nil {
		t.Fatal(err)
	}
	crs.downstream = client.Resource(gvr).Namespace(metav1.NamespaceDefault)

	if err := crs.copySpec("foo"); err == nil {
		t.Errorf("Expected error but got none")
	}
	if err := crs.copySpec("foo"); err != nil {
		t.Errorf("Unexpected error: %s", err)
	}
	if !gock.IsDone() {
		t.Errorf("%d mocks still pending:", len(gock.Pending()))
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
