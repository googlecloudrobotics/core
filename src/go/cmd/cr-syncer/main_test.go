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
	"log"
	"sync"
	"testing"
	"time"

	"github.com/h2non/gock"
	. "github.com/onsi/gomega"
	crdtypes "k8s.io/apiextensions-apiserver/pkg/apis/apiextensions/v1beta1"
	fakecrdclientset "k8s.io/apiextensions-apiserver/pkg/client/clientset/clientset/fake"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/apimachinery/pkg/watch"
	"k8s.io/client-go/dynamic"
	"k8s.io/client-go/rest"
)

func FakeRESTConfig() *rest.Config {
	config := &rest.Config{
		Host: "remote-server",
	}
	return config
}

func FakeClient() dynamic.ResourceInterface {
	client, err := dynamic.NewForConfig(FakeRESTConfig())
	if err != nil {
		log.Fatal(err)
	}
	gvr := schema.GroupVersionResource{
		Group:    "example.com",
		Version:  "v3",
		Resource: "goals",
	}

	return client.Resource(gvr).Namespace("default")
}

func TestCreateOrReplaceSpec_create(t *testing.T) {
	defer gock.Off()

	foo := unstructured.Unstructured{}
	foo.SetAPIVersion("example.com/v3")
	foo.SetKind("Goal")
	foo.SetName("foo")
	foo.SetLabels(map[string]string{"tag": "tagged"})
	foo.SetAnnotations(map[string]string{"annotation": "a"})
	foo.SetResourceVersion("1")
	foo.Object["spec"] = map[string]string{"aSpec": "a"}
	foo.Object["status"] = map[string]string{"aStatus": "a"}
	fooJSON, _ := foo.MarshalJSON()

	target := unstructured.Unstructured{}
	target.UnmarshalJSON(fooJSON)
	delete(target.Object["metadata"].(map[string]interface{}), "resourceVersion")
	target.SetAnnotations(map[string]string{
		"annotation":              "a",
		annotationOwnedByUpstream: "true",
	})
	targetJSON, _ := target.MarshalJSON()

	gock.New("http://remote-server/").
		Get("/apis/example.com/v3/namespaces/default/goals").
		Reply(404)
	gock.New("http://remote-server/").
		Post("/apis/example.com/v3/namespaces/default/goals").
		BodyString(string(targetJSON)).
		Reply(200).
		BodyString(string(targetJSON))

	err := createOrReplaceSpec(FakeClient(), &foo)
	if err != nil {
		t.Errorf("createOrReplaceSpec failed: %v", err)
	}
	if !gock.IsDone() {
		t.Errorf("%d mocks still pending:", len(gock.Pending()))
	}
}

func TestCreateOrReplaceSpec_update(t *testing.T) {
	defer gock.Off()

	foo := unstructured.Unstructured{}
	foo.SetAPIVersion("example.com/v3")
	foo.SetKind("Goal")
	foo.SetName("foo")
	foo.SetLabels(map[string]string{"tag": "tagged"})
	foo.SetAnnotations(map[string]string{
		"annotation":              "a",
		annotationResourceVersion: "1",
	})
	foo.SetResourceVersion("1")
	foo.Object["spec"] = map[string]interface{}{"aSpec": "a"}
	foo.Object["status"] = map[string]interface{}{"aStatus": "a"}

	existing := foo.DeepCopy()
	existing.SetResourceVersion("3")
	existing.SetLabels(map[string]string{"tag": "tagged", "otherlabel": "overwritten"})
	existing.Object["spec"] = map[string]string{"aSpec": "overwritten"}
	existing.Object["status"] = map[string]string{"aStatus": "preserved"}
	existingJSON, _ := existing.MarshalJSON()

	merged := foo.DeepCopy()
	merged.SetResourceVersion("3")
	merged.Object["status"] = existing.Object["status"]
	// Resource version is removed when copying the spec to avoid an infinite loop of edits.
	// The owned-by-remote annotation is added by the syncer.
	merged.SetAnnotations(map[string]string{
		"annotation":              "a",
		annotationOwnedByUpstream: "true",
	})
	mergedJSON, _ := merged.MarshalJSON()

	gock.New("http://remote-server/").
		Get("/apis/example.com/v3/namespaces/default/goals").
		Reply(200).
		BodyString(string(existingJSON))
	gock.New("http://remote-server/").
		Put("/apis/example.com/v3/namespaces/default/goals/foo").
		BodyString(string(mergedJSON)).
		Reply(200).
		BodyString(string(mergedJSON))

	err := createOrReplaceSpec(FakeClient(), &foo)
	if err != nil {
		t.Errorf("createOrReplaceSpec failed: %v", err)
	}
	if !gock.IsDone() {
		t.Errorf("%d mocks still pending:", len(gock.Pending()))
	}
}

func TestCreateOrReplaceSpec_handlesGetError(t *testing.T) {
	defer gock.Off()

	foo := unstructured.Unstructured{}
	foo.SetName("foo")

	gock.New("http://remote-server/").
		Get("/apis/example.com/v3/namespaces/default/goals").
		Reply(500)

	err := createOrReplaceSpec(FakeClient(), &foo)
	if err == nil {
		t.Errorf("createOrReplaceSpec succeeded unexpectedly")
	}
	if !gock.IsDone() {
		t.Errorf("%d mocks still pending:", len(gock.Pending()))
	}
}

func TestCopyStatusHandlesDeletedTarget(t *testing.T) {
	defer gock.Off()

	foo := unstructured.Unstructured{}
	foo.SetName("foo")

	gock.New("http://remote-server/").
		Get("/apis/example.com/v3/namespaces/default/goals/foo").
		Reply(404)

	_, err := tryCopyStatus(FakeClient(), "", &foo)
	if err != nil {
		t.Errorf("tryCopySpec had unexpected error: %v", err)
	}
	if !gock.IsDone() {
		t.Errorf("%d mocks still pending:", len(gock.Pending()))
	}
}

func TestCopyStatusSubtree(t *testing.T) {
	defer gock.Off()

	existing := unstructured.Unstructured{}
	existing.SetAPIVersion("example.com/v3")
	existing.SetKind("Goal")
	existing.SetName("foo")
	existing.Object["status"] = map[string]interface{}{
		"robot": "CREATED",
		"cloud": "RUNNING",
	}
	existingJSON, _ := existing.MarshalJSON()
	source := existing.DeepCopy()
	source.SetResourceVersion("1")
	source.Object["status"] = map[string]interface{}{
		"robot": "RUNNING",
		"cloud": "CREATED",
	}
	target := existing.DeepCopy()
	target.SetAnnotations(map[string]string{
		"cr-syncer.cloudrobotics.com/remote-resource-version": "1",
	})
	target.Object["status"] = map[string]interface{}{
		"robot": "RUNNING",
		"cloud": "RUNNING",
	}
	targetJSON, _ := target.MarshalJSON()

	gock.New("http://remote-server/").
		Get("/apis/example.com/v3/namespaces/default/goals/foo").
		Reply(200).
		BodyString(string(existingJSON))
	gock.New("http://remote-server/").
		Put("/apis/example.com/v3/namespaces/default/goals/foo").
		BodyString(string(targetJSON)).
		Reply(200).
		BodyString(string(targetJSON))

	_, err := tryCopyStatus(FakeClient(), "robot", source)
	if err != nil {
		t.Errorf("tryCopyStatus had unexpected error: %v", err)
	}
	if !gock.IsDone() {
		t.Errorf("%d mocks still pending:", len(gock.Pending()))
	}
}

func TestStreamCrdsSeesPreexistingObject(t *testing.T) {
	g := NewGomegaWithT(t)
	items := []runtime.Object{
		&crdtypes.CustomResourceDefinition{
			ObjectMeta: metav1.ObjectMeta{
				Name:            "foo",
				ResourceVersion: "1",
			},
		},
	}
	cs := fakecrdclientset.NewSimpleClientset(items...)

	var wg sync.WaitGroup
	crds := make(chan CrdChange)
	done := make(chan struct{})
	wg.Add(1)
	go func() {
		defer wg.Done()
		select {
		case crd := <-crds:
			g.Expect(crd.Type).To(Equal(watch.Added))
			g.Expect(crd.CRD.GetName()).To(Equal("foo"))
		case <-time.After(1 * time.Minute):
			t.Errorf("Received no watch event; wanted add for foo")
		}
		close(done)
	}()
	if err := streamCrds(done, cs, crds); err != nil {
		t.Errorf("Got unexpected error: %v", err)
	}
	wg.Wait()
}

func TestStreamCrdsSeesAdditionAndDeletion(t *testing.T) {
	g := NewGomegaWithT(t)
	cs := fakecrdclientset.NewSimpleClientset()

	crds := make(chan CrdChange)
	done := make(chan struct{})
	defer close(done)

	if err := streamCrds(done, cs, crds); err != nil {
		t.Errorf("Got unexpected error: %v", err)
	}

	cs.ApiextensionsV1beta1().CustomResourceDefinitions().Create(&crdtypes.CustomResourceDefinition{
		ObjectMeta: metav1.ObjectMeta{
			Name: "later",
		},
	})
	select {
	case crd := <-crds:
		g.Expect(crd.Type).To(Equal(watch.Added))
		g.Expect(crd.CRD.GetName()).To(Equal("later"))
	case <-time.After(1 * time.Minute):
		t.Errorf("Received no watch event; wanted add for later")
	}

	select {
	case crd := <-crds:
		g.Expect(crd.Type).To(Equal(watch.Modified))
		g.Expect(crd.CRD.GetName()).To(Equal("later"))
	case <-time.After(1 * time.Minute):
		t.Errorf("Received no watch event; wanted deleted for later")
	}

	cs.ApiextensionsV1beta1().CustomResourceDefinitions().Delete("later", &metav1.DeleteOptions{})
	select {
	case crd := <-crds:
		g.Expect(crd.Type).To(Equal(watch.Deleted))
		g.Expect(crd.CRD.GetName()).To(Equal("later"))
	case <-time.After(1 * time.Minute):
		t.Errorf("Received no watch event; wanted deleted for later")
	}
}

func TestStreamCrdsSeesUpdate(t *testing.T) {
	g := NewGomegaWithT(t)
	cs := fakecrdclientset.NewSimpleClientset()

	crds := make(chan CrdChange)
	done := make(chan struct{})
	defer close(done)

	if err := streamCrds(done, cs, crds); err != nil {
		t.Errorf("Got unexpected error: %v", err)
	}

	cs.ApiextensionsV1beta1().CustomResourceDefinitions().Create(&crdtypes.CustomResourceDefinition{
		ObjectMeta: metav1.ObjectMeta{
			Name: "later",
		},
	})
	select {
	case crd := <-crds:
		g.Expect(crd.Type).To(Equal(watch.Added))
		g.Expect(crd.CRD.GetName()).To(Equal("later"))
	case <-time.After(1 * time.Minute):
		t.Errorf("Received no watch event; wanted add for later")
	}

	cs.ApiextensionsV1beta1().CustomResourceDefinitions().Update(&crdtypes.CustomResourceDefinition{
		ObjectMeta: metav1.ObjectMeta{
			Name: "later",
			Annotations: map[string]string{
				"foo": "bar",
			},
		},
	})
	select {
	case crd := <-crds:
		g.Expect(crd.Type).To(Equal(watch.Modified))
		g.Expect(crd.CRD.GetName()).To(Equal("later"))
	case <-time.After(1 * time.Minute):
		t.Errorf("Received no watch event; wanted modified for later")
	}
}
