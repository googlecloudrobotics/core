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

func newGvknn(group, version, kind, namespace, name string) gvknn {
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
	return gvknn{p, group, version, kind, namespace, name}
}

func less(l, r gvknn) bool {
	if l.priority != r.priority {
		return l.priority < r.priority
	}
	if l.group != r.group {
		return l.group < r.group
	}
	if l.version != r.version {
		return l.version < r.version
	}
	if l.kind != r.kind {
		return l.kind < r.kind
	}
	if l.namespace != r.namespace {
		return l.namespace < r.namespace
	}
	return l.name < r.name
}

func gvknnUnstructured(u *unstructured.Unstructured) gvknn {
	gvk := u.GroupVersionKind()
	return newGvknn(gvk.Group, gvk.Version, gvk.Kind, u.GetNamespace(), u.GetName())
}

func gvknnRSpecG(r *apps.ResourceSetSpecGroup) gvknn {
	return newGvknn(r.Group, r.Version, r.Kind, "", "")
}

func gvknnRStatusG(r *apps.ResourceSetStatusGroup) gvknn {
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
