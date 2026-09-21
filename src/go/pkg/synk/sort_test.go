// Copyright 2022 The Cloud Robotics Authors
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
	"reflect"
	"testing"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
)

func TestLessResourceSetStatusGroup(t *testing.T) {
	a := &apps.ResourceSetStatusGroup{
		Group:   "g",
		Version: "v",
		Kind:    "k",
	}
	b := &apps.ResourceSetStatusGroup{
		Group:   "g",
		Version: "v",
		Kind:    "l",
	}
	if !lessResourceSetStatusGroup(a, b) {
		t.Errorf("expected a < b")
	}
	if lessResourceSetStatusGroup(b, a) {
		t.Errorf("expected b >= a")
	}
}

func TestLessResourceSetSpecGroup(t *testing.T) {
	a := &apps.ResourceSetSpecGroup{
		Group:   "g",
		Version: "v",
		Kind:    "k",
	}
	b := &apps.ResourceSetSpecGroup{
		Group:   "g",
		Version: "v",
		Kind:    "l",
	}
	if !lessResourceSetSpecGroup(a, b) {
		t.Errorf("expected a < b")
	}
	if lessResourceSetSpecGroup(b, a) {
		t.Errorf("expected b >= a")
	}
}

func TestLessUnstructured(t *testing.T) {
	a := newUnstructured("v1", "Secret", "ns1", "pod")
	b := newUnstructured("v1", "Secret", "ns1", "poe")
	if !lessUnstructured(a, b) {
		t.Errorf("expected a < b")
	}
	if lessUnstructured(b, a) {
		t.Errorf("expected b >= a")
	}
}

func TestLess(t *testing.T) {
	for _, tc := range []struct {
		a, b *gvknn
	}{
		{newGvknn("a", "v", "k", "ns", "n"), newGvknn("b", "v", "k", "ns", "n")},
		{newGvknn("g", "a", "k", "ns", "n"), newGvknn("g", "b", "k", "ns", "n")},
		{newGvknn("g", "v", "a", "ns", "n"), newGvknn("g", "v", "b", "ns", "n")},
		{newGvknn("g", "v", "k", "a", "n"), newGvknn("g", "v", "k", "b", "n")},
		{newGvknn("g", "v", "k", "ns", "a"), newGvknn("g", "v", "k", "ns", "b")},
		{newGvknn("g", "v", "ServiceAccount", "ns", "a"), newGvknn("g", "v", "Secret", "ns", "b")},
		{newGvknn("g", "v", "Secret", "ns", "a"), newGvknn("g", "v", "ServiceAccount2", "ns", "b")},
		{newGvknn("v1", "v", "Secret", "ns", "a"), newGvknn("rbac.authorization.k8s.io", "v1", "Role", "ns", "b")},
		{newGvknn("rbac.authorization.k8s.io", "v1", "ClusterRole", "", "a"), newGvknn("rbac.authorization.k8s.io", "v1", "ClusterRoleBinding", "", "b")},
		{newGvknn("rbac.authorization.k8s.io", "v1", "Role", "ns", "a"), newGvknn("rbac.authorization.k8s.io", "v1", "RoleBinding", "ns", "b")},
		{newGvknn("rbac.authorization.k8s.io", "v1", "RoleBinding", "ns", "a"), newGvknn("", "v1", "ConfigMap", "ns", "b")},
		{newGvknn("rbac.authorization.k8s.io", "v1", "ClusterRoleBinding", "", "a"), newGvknn("apps", "v1", "Deployment", "ns", "b")},
	} {
		if !less(tc.a, tc.b) {
			t.Errorf("expected a (%v) < b (%v)", tc.a, tc.b)
		}
		if less(tc.b, tc.a) {
			t.Errorf("expected b (%v) >= a (%v)", tc.b, tc.a)
		}
	}
}

func TestSortResources_RBACOrder(t *testing.T) {
	ns := newUnstructured("v1", "Namespace", "", "ns1")
	sa := newUnstructured("v1", "ServiceAccount", "ns1", "sa1")
	secret := newUnstructured("v1", "Secret", "ns1", "secret1")
	clusterRole := newUnstructured("rbac.authorization.k8s.io/v1", "ClusterRole", "", "cr1")
	role := newUnstructured("rbac.authorization.k8s.io/v1", "Role", "ns1", "role1")
	clusterRoleBinding := newUnstructured("rbac.authorization.k8s.io/v1", "ClusterRoleBinding", "", "crb1")
	roleBinding := newUnstructured("rbac.authorization.k8s.io/v1", "RoleBinding", "ns1", "rb1")
	cm := newUnstructured("v1", "ConfigMap", "ns1", "cm1")
	deploy := newUnstructured("apps/v1", "Deployment", "ns1", "deploy1")

	resources := []*unstructured.Unstructured{
		deploy, roleBinding, cm, role, clusterRoleBinding, secret, clusterRole, sa, ns,
	}
	sortResources(resources)

	want := []*unstructured.Unstructured{
		ns, sa, secret, clusterRole, role, clusterRoleBinding, roleBinding, cm, deploy,
	}
	if !reflect.DeepEqual(resources, want) {
		t.Errorf("sortResources() mismatch:\ngot:  %v\nwant: %v", resources, want)
	}
}
