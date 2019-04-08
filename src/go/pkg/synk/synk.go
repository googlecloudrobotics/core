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

// Package synk contains functionality to synchronize a batch of resources
// with a cluster while correctly handling CRDs and deletions.
package synk

import (
	"context"
	"encoding/json"
	"fmt"
	"reflect"
	"regexp"
	"sort"
	"strconv"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"github.com/pkg/errors"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/client-go/dynamic"
	"k8s.io/client-go/rest"
)

// Synk allows to synchronize sets of resources with a fixed cluster.
type Synk struct {
	restcfg *rest.Config
	client  dynamic.Interface
}

// New returns a new Synk object that acts against the cluster for the given configuration.
func New(cfg *rest.Config) (*Synk, error) {
	c, err := dynamic.NewForConfig(cfg)
	if err != nil {
		return nil, err
	}
	return &Synk{
		restcfg: cfg,
		client:  c,
	}, nil
}

// TODO: determine options that allow us to be semantically compatible with
// vanilla kubectl apply.
type ApplyOptions struct {
	name    string
	version int32
}

func (s *Synk) Apply(
	ctx context.Context,
	name string,
	opts *ApplyOptions,
	resources ...*unstructured.Unstructured,
) (*apps.ResourceSet, error) {
	if opts == nil {
		opts = &ApplyOptions{}
	}
	opts.name = name

	// Cleanup and sort resources.
	resources = filter(resources, func(r *unstructured.Unstructured) bool {
		return !reflect.DeepEqual(*r, unstructured.Unstructured{})
	})
	sortResources(resources)

	regulars := filter(resources, func(r *unstructured.Unstructured) bool {
		return !isCustomResourceDefinition(r)
	})
	crds := filter(resources, isCustomResourceDefinition)
	_, _ = regulars, crds

	// Initialize and create next ResourceSet.
	var err error
	opts.version, err = s.next(ctx, opts.name)
	if err != nil {
		return nil, errors.Wrap(err, "get next ResourceSet version")
	}

	var rs apps.ResourceSet
	rs.Name = resourceSetName(opts.name, opts.version)
	rs.Labels = map[string]string{"name": opts.name}

	groupedResources := map[schema.GroupVersionKind][]apps.ResourceRef{}
	for _, r := range resources {
		gvk := r.GroupVersionKind()
		groupedResources[gvk] = append(groupedResources[gvk], apps.ResourceRef{
			Namespace: r.GetNamespace(),
			Name:      r.GetName(),
		})
	}
	for gvk, res := range groupedResources {
		rs.Spec.Resources = append(rs.Spec.Resources, apps.ResourceSetSpecGroup{
			Group:   gvk.Group,
			Version: gvk.Version,
			Kind:    gvk.Kind,
			Items:   res,
		})
	}
	sort.Slice(rs.Spec.Resources, func(i, j int) bool {
		a, b := rs.Spec.Resources[i], rs.Spec.Resources[j]
		return a.Group < b.Group && a.Version < b.Version && a.Kind < b.Kind
	})

	rs.Status = apps.ResourceSetStatus{
		Phase:     apps.ResourceSetPhasePending,
		StartedAt: metav1.Now(),
	}
	if err := s.createResourceSet(&rs); err != nil {
		return nil, errors.Wrapf(err, "create resources object %q", rs.Name)
	}

	// Set ResourceSet owner on all resources and initialize their status.
	status := map[string]*apps.ResourceStatus{}

	for _, r := range resources {
		if err := s.setOwnerRef(&rs, r, opts); err != nil {
			return nil, errors.Wrapf(err, "set owner reference for %q", resourceKey(r))
		}
		status[resourceKey(r)] = &apps.ResourceStatus{
			Namespace: r.GetNamespace(),
			Name:      r.GetName(),
			Action:    apps.ResourceActionNone,
		}
	}
	return &rs, nil
}

var resourceSetGVR = schema.GroupVersionResource{
	Group:    "apps.cloudrobotics.com",
	Version:  "v1alpha1",
	Resource: "resourcesets",
}

func (s *Synk) createResourceSet(rs *apps.ResourceSet) error {
	rs.Kind = "ResourceSet"
	rs.APIVersion = "apps.cloudrobotics.com/v1alpha1"

	var u unstructured.Unstructured
	if err := convert(rs, &u); err != nil {
		return err
	}
	res, err := s.client.Resource(resourceSetGVR).Create(&u, metav1.CreateOptions{})
	if err != nil {
		return err
	}
	return convert(res, rs)
}

// next returns the next version for the resources name.
func (s *Synk) next(ctx context.Context, name string) (version int32, err error) {
	list, err := s.client.Resource(resourceSetGVR).List(metav1.ListOptions{})
	if err != nil {
		return 0, errors.Wrap(err, "list existing ResourceSets")
	}
	var curVersion int32
	for _, r := range list.Items {
		n, v, ok := decodeResourceSetName(r.GetName())
		if !ok || n != name {
			continue
		}
		if v > curVersion {
			curVersion = v
		}
	}
	return curVersion + 1, nil
}

// setOwnerRef sets the owning ResourceSet of the resource. Older ResourceSet owners
// for the same name are overwritten.
// If other conflicting ResourceSet owners are already set, an error is returned.
func (s *Synk) setOwnerRef(rs *apps.ResourceSet, resource *unstructured.Unstructured, opts *ApplyOptions) error {
	// An object must have at most one owner reference to a Resources object.
	var ownerRefs []metav1.OwnerReference
	for _, or := range ownerRefs {
		if or.APIVersion != "apps.cloudrobotics.com/v1alpha1" || or.Kind != "Resources" {
			ownerRefs = append(ownerRefs, or)
			continue
		}
		name, version, ok := decodeResourceSetName(or.Name)
		if !ok {
			return errors.Errorf("Resources owner reference with invalid name %q", or.Name)
		}
		if name != opts.name {
			return errors.Errorf("owned by conflicting Resources object %q", or.Name)
		}
		if version > opts.version {
			return errors.Errorf("conflicting Resources version %d", version)
		}
		// Valid reference to previous Resources object, skip it.
	}
	_true := true
	ownerRefs = append(ownerRefs, metav1.OwnerReference{
		APIVersion:         "apps.cloudrobotics.com/v1alpha1",
		Kind:               "Resources",
		Name:               rs.Name,
		UID:                rs.UID,
		BlockOwnerDeletion: &_true,
	})
	resource.SetOwnerReferences(ownerRefs)
	return nil
}

func isCustomResourceDefinition(r *unstructured.Unstructured) bool {
	return r.GetAPIVersion() == "apiextensions.k8s.io/v1beta1" && r.GetKind() == "CustomResourceDefinition"
}

func resourceSetName(s string, v int32) string {
	return fmt.Sprintf("%s.v%d", s, v)
}

var namePat = regexp.MustCompile(`^([a-z0-9]+)\.v([0-9]+)$`)

func decodeResourceSetName(s string) (string, int32, bool) {
	res := namePat.FindStringSubmatch(s)
	if len(res) == 0 {
		return "", 0, false
	}
	version, err := strconv.Atoi(res[2])
	if err != nil {
		panic(err)
	}
	return res[1], int32(version), true
}

func sortResources(res []*unstructured.Unstructured) {
	sort.Slice(res, func(i, j int) (b bool) {
		return resourceKey(res[i]) < resourceKey(res[j])
	})
}

func resourceKey(r *unstructured.Unstructured) string {
	return fmt.Sprintf("%s/%s/%s/%s",
		r.GetAPIVersion(),
		r.GetKind(),
		r.GetNamespace(),
		r.GetName(),
	)
}

func filter(in []*unstructured.Unstructured, f func(*unstructured.Unstructured) bool) (out []*unstructured.Unstructured) {
	for _, r := range in {
		if f(r) {
			out = append(out, r)
		}
	}
	return out
}

// convert a resource from one type representation to another one.
func convert(from, to runtime.Object) error {
	b, err := json.Marshal(from)
	if err != nil {
		return err
	}
	return json.Unmarshal(b, &to)
}
