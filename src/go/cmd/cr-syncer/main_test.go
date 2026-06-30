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
	"sync"
	"testing"
	"time"

	crdtypes "k8s.io/apiextensions-apiserver/pkg/apis/apiextensions/v1"
	fakecrdclientset "k8s.io/apiextensions-apiserver/pkg/client/clientset/clientset/fake"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/watch"
)

func TestStreamCrdsSeesPreexistingObject(t *testing.T) {
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
			if crd.Type != watch.Added {
				t.Errorf("crd.Type = %v; want %v", crd.Type, watch.Added)
			}
			if name := crd.CRD.GetName(); name != "foo" {
				t.Errorf("crd.CRD.GetName() = %q; want %q", name, "foo")
			}
		case <-time.After(15 * time.Second):
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
	ctx := t.Context()
	cs := fakecrdclientset.NewSimpleClientset()

	crds := make(chan CrdChange)
	done := make(chan struct{})
	defer close(done)

	if err := streamCrds(done, cs, crds); err != nil {
		t.Errorf("Got unexpected error: %v", err)
	}

	cs.ApiextensionsV1().CustomResourceDefinitions().Create(ctx,
		&crdtypes.CustomResourceDefinition{
			ObjectMeta: metav1.ObjectMeta{
				Name: "later",
			},
		},
		metav1.CreateOptions{})
	select {
	case crd := <-crds:
		if crd.Type != watch.Added {
			t.Errorf("crd.Type = %v; want %v", crd.Type, watch.Added)
		}
		if name := crd.CRD.GetName(); name != "later" {
			t.Errorf("crd.CRD.GetName() = %q; want %q", name, "later")
		}
	case <-time.After(15 * time.Second):
		t.Errorf("Received no watch event; wanted add for later")
	}

	cs.ApiextensionsV1().CustomResourceDefinitions().Delete(ctx, "later", metav1.DeleteOptions{})
	select {
	case crd := <-crds:
		if crd.Type != watch.Deleted {
			t.Errorf("crd.Type = %v; want %v", crd.Type, watch.Deleted)
		}
		if name := crd.CRD.GetName(); name != "later" {
			t.Errorf("crd.CRD.GetName() = %q; want %q", name, "later")
		}
	case <-time.After(15 * time.Second):
		t.Errorf("Received no watch event; wanted deleted for later")
	}
}

func TestStreamCrdsSeesUpdate(t *testing.T) {
	ctx := t.Context()
	cs := fakecrdclientset.NewSimpleClientset()

	crds := make(chan CrdChange)
	done := make(chan struct{})
	defer close(done)

	if err := streamCrds(done, cs, crds); err != nil {
		t.Errorf("Got unexpected error: %v", err)
	}

	cs.ApiextensionsV1().CustomResourceDefinitions().Create(ctx,
		&crdtypes.CustomResourceDefinition{
			ObjectMeta: metav1.ObjectMeta{
				Name: "later",
			},
		},
		metav1.CreateOptions{})
	select {
	case crd := <-crds:
		if crd.Type != watch.Added {
			t.Errorf("crd.Type = %v; want %v", crd.Type, watch.Added)
		}
		if name := crd.CRD.GetName(); name != "later" {
			t.Errorf("crd.CRD.GetName() = %q; want %q", name, "later")
		}
	case <-time.After(15 * time.Second):
		t.Errorf("Received no watch event; wanted add for later")
	}

	cs.ApiextensionsV1().CustomResourceDefinitions().Update(ctx,
		&crdtypes.CustomResourceDefinition{
			ObjectMeta: metav1.ObjectMeta{
				Name: "later",
				Annotations: map[string]string{
					"foo": "bar",
				},
			},
		},
		metav1.UpdateOptions{})
	select {
	case crd := <-crds:
		if crd.Type != watch.Modified {
			t.Errorf("crd.Type = %v; want %v", crd.Type, watch.Modified)
		}
		if name := crd.CRD.GetName(); name != "later" {
			t.Errorf("crd.CRD.GetName() = %q; want %q", name, "later")
		}
	case <-time.After(15 * time.Second):
		t.Errorf("Received no watch event; wanted modified for later")
	}
}
