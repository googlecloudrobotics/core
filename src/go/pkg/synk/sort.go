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
	"fmt"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
)

// gvknn is only used to unify the less functions.
type gvknn struct {
	priority  int
	group     string
	version   string
	kind      string
	namespace string
	name      string
}

func newGvknn(group, version, kind, namespace, name string) *gvknn {
	p := 999
	switch kind {
	case "Namespace":
		// Adding resources to a non existing namespace removes them. So namespaces
		// need to go early.
		p = 1
	case "ServiceAccount":
		p = 2
	case "Secret":
		// We need ServiceAccount to be before Secret. The token controller removes
		// Secrets with non existing ServiceAccount.
		p = 3
	}
	return &gvknn{p, group, version, kind, namespace, name}
}

func less(l, r *gvknn) bool {
	ls := fmt.Sprintf("%03d/%s/%s/%s/%s/%s", l.priority, l.group, l.version, l.kind, l.namespace, l.name)
	rs := fmt.Sprintf("%03d/%s/%s/%s/%s/%s", r.priority, r.group, r.version, r.kind, r.namespace, r.name)
	return ls < rs
}

func gvknnUnstructured(u *unstructured.Unstructured) *gvknn {
	gvk := u.GroupVersionKind()
	return newGvknn(gvk.Group, gvk.Version, gvk.Kind, u.GetNamespace(), u.GetName())
}

func gvknnRSpecG(r *apps.ResourceSetSpecGroup) *gvknn {
	return newGvknn(r.Group, r.Version, r.Kind, "", "")
}

func gvknnRStatusG(r *apps.ResourceSetStatusGroup) *gvknn {
	return newGvknn(r.Group, r.Version, r.Kind, "", "")
}

func lessUnstructured(l, r *unstructured.Unstructured) bool {
	return less(gvknnUnstructured(l), gvknnUnstructured(r))
}

func lessResourceSetSpecGroup(l, r *apps.ResourceSetSpecGroup) bool {
	return less(gvknnRSpecG(l), gvknnRSpecG(r))
}

func lessResourceSetStatusGroup(l, r *apps.ResourceSetStatusGroup) bool {
	return less(gvknnRStatusG(l), gvknnRStatusG(r))
}
