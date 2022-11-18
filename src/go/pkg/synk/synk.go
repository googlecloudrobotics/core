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
	"log"
	"reflect"
	"regexp"
	"sort"
	"strconv"
	"strings"
	"time"

	"github.com/cenkalti/backoff"
	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"
	corev1 "k8s.io/api/core/v1"
	apiextensions "k8s.io/apiextensions-apiserver/pkg/apis/apiextensions/v1"
	k8serrors "k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/api/meta"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/apimachinery/pkg/util/jsonmergepatch"
	"k8s.io/apimachinery/pkg/util/mergepatch"
	"k8s.io/apimachinery/pkg/util/strategicpatch"
	"k8s.io/client-go/discovery"
	cacheddiscovery "k8s.io/client-go/discovery/cached"
	"k8s.io/client-go/dynamic"
	"k8s.io/client-go/kubernetes/scheme"
	"k8s.io/client-go/rest"
	"k8s.io/client-go/restmapper"
)

// src/k8s.io/apimachinery/pkg/api/validation/objectmeta.go
const totalAnnotationSizeLimitB int = 256 * (1 << 10) // 256 kB

// Synk allows to synchronize sets of resources with a fixed cluster.
type Synk struct {
	discovery   discovery.CachedDiscoveryInterface
	client      dynamic.Interface
	mapper      meta.RESTMapper
	resetMapper func()
}

// New returns a new Synk object that acts against the cluster for the given configuration.
func New(client dynamic.Interface, discovery discovery.CachedDiscoveryInterface) *Synk {
	s := &Synk{
		discovery: discovery,
		client:    client,
	}
	// Store reset function seperately to allow reasonable tests.
	m := restmapper.NewDeferredDiscoveryRESTMapper(discovery)
	s.mapper = m
	s.resetMapper = m.Reset

	return s
}

func NewForConfig(cfg *rest.Config) (*Synk, error) {
	client, err := dynamic.NewForConfig(cfg)
	if err != nil {
		return nil, err
	}
	discovery, err := discovery.NewDiscoveryClientForConfig(cfg)
	if err != nil {
		return nil, err
	}
	cachedDiscovery := cacheddiscovery.NewMemCacheClient(discovery)
	// Without initial invalidation all calls will fail.
	cachedDiscovery.Invalidate()

	return New(client, cachedDiscovery), nil
}

// TODO: determine options that allow us to be semantically compatible with
// vanilla kubectl apply.
type ApplyOptions struct {
	name    string
	version int32

	// Namespace that's set for all namespaced resources that have no
	// other namespace set yet.
	Namespace string
	// EnforceNamespace causes apply to fail if a resource has a namespace set
	// that's different from Namespace.
	EnforceNamespace bool

	// Log functions to report progress and failures while applying resources.
	Log func(r *unstructured.Unstructured, a apps.ResourceAction, status, msg string)
}

const (
	StatusSuccess = "success"
	StatusFailure = "failure"
)

func (o *ApplyOptions) logf(r *unstructured.Unstructured, action apps.ResourceAction, msg string, args ...interface{}) {
	if o.Log != nil {
		o.Log(r, action, StatusSuccess, fmt.Sprintf(msg, args...))
	}
}

func (o *ApplyOptions) errorf(r *unstructured.Unstructured, action apps.ResourceAction, msg string, args ...interface{}) {
	if o.Log != nil {
		o.Log(r, action, StatusFailure, fmt.Sprintf(msg, args...))
	}
}

// Init installs the ResourceSet CRD into the cluster and waits for
// it to become available.
// It does not need to be called before each use of Synk.
func (s *Synk) Init() error {
	vTrue := true
	crd := &apiextensions.CustomResourceDefinition{
		TypeMeta: metav1.TypeMeta{
			APIVersion: "apiextensions.k8s.io/v1",
			Kind:       "CustomResourceDefinition",
		},
		ObjectMeta: metav1.ObjectMeta{
			Name: "resourcesets.apps.cloudrobotics.com",
		},
		Spec: apiextensions.CustomResourceDefinitionSpec{
			Group: "apps.cloudrobotics.com",
			Names: apiextensions.CustomResourceDefinitionNames{
				Kind:     "ResourceSet",
				Plural:   "resourcesets",
				Singular: "resourceset",
			},
			Scope: apiextensions.ClusterScoped,
			Versions: []apiextensions.CustomResourceDefinitionVersion{{
				Name:    "v1alpha1",
				Served:  true,
				Storage: true,
				// TODO(ensonic): replace with the actual schema
				Schema: &apiextensions.CustomResourceValidation{
					OpenAPIV3Schema: &apiextensions.JSONSchemaProps{
						Type: "object",
						Properties: map[string]apiextensions.JSONSchemaProps{
							"spec": {
								Type:                   "object",
								XPreserveUnknownFields: &vTrue,
							},
							"status": {
								Type:                   "object",
								XPreserveUnknownFields: &vTrue,
							},
						},
					},
				},
			}},
		},
	}
	var u unstructured.Unstructured
	if err := convert(crd, &u); err != nil {
		return err
	}
	if _, err := s.applyOne(context.Background(), &u, nil); err != nil {
		return errors.Wrap(err, "create ResourceSet CRD")
	}

	err := backoff.Retry(
		func() error {
			s.discovery.Invalidate()
			ok, err := s.crdAvailable(&u)
			if err != nil {
				return err
			}
			if !ok {
				return errors.New("crd not available")
			}
			return nil
		},
		backoff.WithMaxRetries(backoff.NewConstantBackOff(2*time.Second), 60),
	)
	if err != nil {
		return errors.Wrap(err, "wait for ResourceSet CRD")
	}

	return nil
}

// Delete removes the resources that are part of the ResourceSet specified by
// 'name'. It uses so-called "foreground cascading deletion", which means that:
//
// - it returns after marking the ResourceSet for deletion, but before the
//
//	resources have been deleted
//
// - the ResourceSet will not be deleted until all resources have been deleted
//
// This ensures that if a new ResourceSet is created before all resources have
// been deleted, it will have a higher version number.
func (s *Synk) Delete(ctx context.Context, name string) error {
	policy := metav1.DeletePropagationForeground
	deleteOpts := metav1.DeleteOptions{PropagationPolicy: &policy}
	return s.client.Resource(resourceSetGVR).DeleteCollection(ctx, deleteOpts, metav1.ListOptions{
		LabelSelector: fmt.Sprintf("name=%s", name),
	})
}

// Apply installs or updates the ResourceSet specified by 'name'.
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

	// applyAll() updates the resources in place. To avoid modifying the
	// caller's slice, copy the resources first.
	resources = append([]*unstructured.Unstructured(nil), resources...)
	for i, r := range resources {
		resources[i] = r.DeepCopy()
	}

	rs, resources, err := s.initialize(ctx, opts, resources...)
	if err != nil {
		return rs, err
	}
	results, applyErr := s.applyAll(ctx, rs, opts, resources...)

	if err := s.updateResourceSetStatus(ctx, rs, results); err != nil {
		return rs, err
	}
	if applyErr == nil {
		if err := s.deleteResourceSets(ctx, opts.name, opts.version); err != nil {
			return rs, err
		}
	}
	return rs, applyErr
}

type transientErr struct {
	error
}

// IsTransientErr returns true if the error may resolve by retrying the operation.
func IsTransientErr(err error) bool {
	// Either a custom error is specifically wrapped in transientErr or the innermost
	// error is a known transient Kubernetes API error.
	_, ok1 := err.(transientErr)
	_, ok2 := err.(*transientErr)
	if ok1 || ok2 {
		return true
	}
	err = errors.Cause(err)
	switch {
	// May happen on resourceVersion mismatches or patch conflicts.
	case k8serrors.IsConflict(err):
	case k8serrors.IsResourceExpired(err):
	// May happen if a created object has already been created.
	case k8serrors.IsAlreadyExists(err):
	// May happen if a patched resource has already been deleted.
	case k8serrors.IsNotFound(err):
	case k8serrors.IsGone(err):
	// Server-side transient errors.
	case k8serrors.IsInternalError(err):
	case k8serrors.IsServerTimeout(err):
	case k8serrors.IsTimeout(err):
	case k8serrors.IsTooManyRequests(err):
	case k8serrors.IsServiceUnavailable(err):
	// May happen shortly after CRD creation.
	case discovery.IsGroupDiscoveryFailedError(err):
	// May happen if a chart is deleted and immediately recreated.
	// https://github.com/kubernetes/kubernetes/blob/d2a081c8e14e21e28fe5bdfa38a817ef9c0bb8e3/staging/src/k8s.io/apiserver/pkg/admission/plugin/namespace/lifecycle/admission.go#L173
	case strings.Contains(err.Error(), "unable to create new content in namespace"):
	// Note: Golang switch cases don't fall through, so we only return
	// false in the default case.
	default:
		return false
	}
	return true
}

func (s *Synk) applyAll(
	ctx context.Context,
	rs *apps.ResourceSet,
	opts *ApplyOptions,
	resources ...*unstructured.Unstructured,
) (applyResults, error) {
	results := applyResults{}

	crds, regulars := separateCRDsFromResources(resources)

	// Insert CRDs and wait for them to become available.
	for _, crd := range crds {
		// CRDs must never be replaced as deleting them will delete
		// all its current instances. Update conflicts must be resolved manually.
		action, err := s.applyOne(ctx, crd, rs)
		if err != nil {
			opts.errorf(crd, action, "failed to apply: %s", err)
		} else {
			opts.logf(crd, action, "applied successfully")
		}
		results.set(crd, action, err)
	}
	err := backoff.Retry(
		func() error {
			s.discovery.Invalidate()
			for _, crd := range crds {
				if ok, err := s.crdAvailable(crd); err != nil {
					return backoff.Permanent(err)
				} else if !ok {
					return fmt.Errorf("crd not yet available: %q", crd.GetName())
				}
			}
			return nil
		},
		backoff.WithMaxRetries(backoff.NewConstantBackOff(2*time.Second), 60),
	)
	if err != nil {
		return results, errors.Wrap(err, "wait for CRDs")
	}
	// Reset all discovery and mapping once again.
	s.resetMapper()

	// Try applying until the errors stay the same between iterations. Put in
	// an upper bound just in case of flapping errors.
	prevFailures := 0

	for i := 0; i < 10; i++ {
		curFailures := 0

		for _, r := range regulars {
			// Don't retry resources that were applied successfully
			// in the first iteration.
			if i > 0 && !results.failed(r) {
				continue
			}
			// Attach the ResourceSet as owner. CRDs are exempt since
			// the risk of unintended deletion of all its instances is too high.
			setOwnerRef(r, rs)
			action, err := s.applyOne(ctx, r, rs)
			if err != nil {
				curFailures++
				opts.errorf(r, action, "failed to apply, may retry: %s", err)
			} else {
				opts.logf(r, action, "applied successfully")
			}
			results.set(r, action, err)
		}
		if curFailures == 0 || curFailures == prevFailures {
			break
		}
		prevFailures = curFailures
	}
	// The overall error we return is a transient error if all resource errors
	// are transient. If there's at least one permanent failure, retrying
	// will never make Apply overall successful.
	allTransient := true
	numErrors := 0
	var firstFailure *applyResult
	for _, r := range results {
		if r.err != nil {
			if !IsTransientErr(r.err) {
				allTransient = false
			}
			if firstFailure == nil {
				firstFailure = r
			}
			numErrors++
		}
	}
	if numErrors == 0 {
		return results, nil
	}
	err = fmt.Errorf("%d/%d resources failed to apply", numErrors, len(results))
	if numErrors == 1 {
		err = fmt.Errorf("%s: %s: %s", err, resourceKey(firstFailure.resource), firstFailure.err)
	} else {
		err = fmt.Errorf("%s, including %s: %s", err, resourceKey(firstFailure.resource), firstFailure.err)
	}
	if allTransient {
		err = transientErr{err}
	}
	return results, err
}

// initialize a new ResourceSet version for the given name and prepare resources
// for it.
func (s *Synk) initialize(
	ctx context.Context,
	opts *ApplyOptions,
	resources ...*unstructured.Unstructured,
) (*apps.ResourceSet, []*unstructured.Unstructured, error) {
	// Cleanup and sort resources.
	resources = filter(resources, func(r *unstructured.Unstructured) bool {
		return !reflect.DeepEqual(*r, unstructured.Unstructured{}) && !isTestResource(r)
	})
	sortResources(resources)

	crds, regulars := separateCRDsFromResources(resources)

	if err := s.populateNamespaces(ctx, opts.Namespace, crds, regulars...); err != nil {
		return nil, nil, errors.Wrap(err, "set default namespaces")
	}
	// TODO: consider putting this and other validation as a step after initialize
	// so we can give validation errors in batch in the ResourceSet status.
	if opts.EnforceNamespace {
		for _, r := range regulars {
			if ns := r.GetNamespace(); ns != "" && ns != opts.Namespace && ns != "kube-system" {
				return nil, nil, errors.Errorf("invalid namespace %q on %q, expected %q or \"kube-system\"", ns, resourceKey(r), opts.Namespace)
			}
		}
	}

	// Initialize and create next ResourceSet.
	var err error
	opts.version, err = s.next(ctx, opts.name)
	if err != nil {
		return nil, nil, errors.Wrap(err, "get next ResourceSet version")
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
		return lessResourceSetSpecGroup(&rs.Spec.Resources[i], &rs.Spec.Resources[j])
	})

	rs.Status = apps.ResourceSetStatus{
		Phase:     apps.ResourceSetPhasePending,
		StartedAt: metav1.Now(),
	}
	if err := s.createResourceSet(ctx, &rs); err != nil {
		return nil, nil, errors.Wrapf(err, "create resources object %q", rs.Name)
	}

	return &rs, resources, nil
}

// Set default namespace on all namespaced resources.
func (s *Synk) populateNamespaces(
	ctx context.Context,
	ns string,
	crds []*unstructured.Unstructured,
	resources ...*unstructured.Unstructured,
) error {
	_, span := trace.StartSpan(ctx, "Discover server resources")
	_, list, err := s.discovery.ServerGroupsAndResources()
	span.End()
	if err != nil {
		return errors.Wrap(err, "discover server resources")
	}
	// We have to consider discoverable resources as well as CRDs that
	// will only be added later.
	isNamespaced := map[string]bool{}

	for _, srvRes := range list {
		for _, sr := range srvRes.APIResources {
			isNamespaced[srvRes.GroupVersion+"/"+sr.Kind] = sr.Namespaced
		}
	}
	for _, crd := range crds {
		var typed apiextensions.CustomResourceDefinition
		if err := convert(crd, &typed); err != nil {
			return errors.Wrapf(err, "invalid CustomResourceDefinition %q", resourceKey(crd))
		}
		for _, v := range typed.Spec.Versions {
			k := typed.Spec.Group + "/" + v.Name + "/" + typed.Spec.Names.Kind
			isNamespaced[k] = typed.Spec.Scope != apiextensions.ClusterScoped
		}
	}
	for _, r := range resources {
		if r.GetNamespace() == "" && isNamespaced[r.GetAPIVersion()+"/"+r.GetKind()] {
			r.SetNamespace(ns)
		}
	}
	return nil
}

func deleteAppliedAnnotation(u *unstructured.Unstructured) {
	anns := u.GetAnnotations()
	if anns == nil {
		anns = map[string]string{}
	}
	// Delete any potential pre-existing annotation.
	delete(anns, corev1.LastAppliedConfigAnnotation)
	u.SetAnnotations(anns)
}

func setAppliedAnnotation(u *unstructured.Unstructured) error {
	deleteAppliedAnnotation(u)

	b, err := u.MarshalJSON()
	if err != nil {
		return err
	}
	if len(b) >= totalAnnotationSizeLimitB {
		return errors.Errorf("skipping annotation %q for %q: size %d > max. allowed size %d",
			corev1.LastAppliedConfigAnnotation, u.GetName(), len(b), totalAnnotationSizeLimitB)
	}
	anns := u.GetAnnotations()
	anns[corev1.LastAppliedConfigAnnotation] = string(b)
	u.SetAnnotations(anns)
	return nil
}

func getAppliedAnnotation(u *unstructured.Unstructured) []byte {
	return []byte(u.GetAnnotations()[corev1.LastAppliedConfigAnnotation])
}

// validateOwnerRefs returns an error if the resource has ResourceSet owners
// that are not predecessors of name/version.
func validateOwnerRefs(r *unstructured.Unstructured, set *apps.ResourceSet) error {
	if set == nil {
		return nil
	}
	name, version, ok := decodeResourceSetName(set.Name)
	if !ok {
		return errors.Errorf("invalid ResourceSet name %q", set.Name)
	}
	for _, or := range r.GetOwnerReferences() {
		if or.APIVersion != "apps.cloudrobotics.com/v1alpha1" || or.Kind != "ResourceSet" {
			continue
		}
		n, v, ok := decodeResourceSetName(or.Name)
		if !ok {
			return errors.Errorf("ResourceSet owner reference with invalid name %q", or.Name)
		}
		if n != name {
			return errors.Errorf("owned by conflicting ResourceSet object %q", or.Name)
		}
		if v > version {
			// TODO(rodrigoq): should this be transient to cope with concurrent synk runs?
			return errors.Errorf("owned by newer ResourceSet %q > v%d", or.Name, version)
		}
	}
	return nil
}

// setOwnerRef sets the ResourceSet as the owner and removers all other ResourceSet
// owner references.
func setOwnerRef(r *unstructured.Unstructured, set *apps.ResourceSet) {
	var newRefs []metav1.OwnerReference
	for _, or := range r.GetOwnerReferences() {
		if or.APIVersion != "apps.cloudrobotics.com/v1alpha1" || or.Kind != "ResourceSet" {
			newRefs = append(newRefs, or)
		}
	}
	_true := true
	newRefs = append(newRefs, metav1.OwnerReference{
		APIVersion:         "apps.cloudrobotics.com/v1alpha1",
		Kind:               "ResourceSet",
		Name:               set.Name,
		UID:                set.UID,
		BlockOwnerDeletion: &_true,
	})
	r.SetOwnerReferences(newRefs)
}

// canReplace determines whether an "apply patch/update" error is likely to be
// resolved by deleting and recreating the resource. Some resources have
// immutable fields (eg Job.spec.template) that can only be changed this way.
// This is analogous to `kubectl apply --force`.
func canReplace(resource *unstructured.Unstructured, patchErr error) bool {
	k := resource.GetKind()
	e := patchErr.Error()
	if (k == "DaemonSet" || k == "Deployment" || k == "Job") && strings.Contains(e, "field is immutable") {
		return true
	}
	if k == "Service" && (strings.Contains(e, "field is immutable") || strings.Contains(e, "may not change once set") || strings.Contains(e, "can not be unset")) {
		return true
	}
	if k == "PersistentVolume" && strings.Contains(e, "is immutable after creation") {
		v, ok, err := unstructured.NestedString(resource.Object, "spec", "persistentVolumeReclaimPolicy")
		if err == nil && ok && v == "Retain" {
			return true
		}
		log.Printf("Not replacing PersistentVolume since reclaim policy is not Retain but %q", v)
	}
	if (k == "ValidatingWebhookConfiguration" || k == "MutatingWebhookConfiguration") && strings.Contains(e, "must be specified for an update") {
		return true
	}

	// TODO(rodrigoq): can other resources be safely replaced?
	return false
}

func replace(ctx context.Context, client dynamic.ResourceInterface, resource *unstructured.Unstructured) (*unstructured.Unstructured, error) {
	// Foreground deletion means that the new job can't be created until the old
	// pods are gone, so updates to a currently-running job are safer.
	policy := metav1.DeletePropagationForeground
	deleteOpts := metav1.DeleteOptions{PropagationPolicy: &policy}
	if err := client.Delete(ctx, resource.GetName(), deleteOpts); err != nil {
		return nil, errors.Wrap(err, "delete")
	}
	res, err := client.Create(ctx, resource, metav1.CreateOptions{})
	if err != nil {
		// This is likely to occur if deletion is not immediate, in which case
		// this returns a transient AlreadyExists error, and the outer loop will
		// retry until the resource is deleted.
		return nil, errors.Wrap(err, "create")
	}
	return res, nil
}

func (s *Synk) applyOne(ctx context.Context, resource *unstructured.Unstructured, set *apps.ResourceSet) (apps.ResourceAction, error) {
	// If name is unset, we'd retrieve a list below and panic.
	// TODO: This may be valid if generateName is set instead. In this case we
	// want to create the resource in any case.
	if resource.GetName() == "" {
		return apps.ResourceActionNone, errors.New("missing resource name")
	}
	ctx, span := trace.StartSpan(ctx, "Apply "+resource.GetName())
	defer span.End()
	// GroupVersionKind is not sufficient to determine the REST API path to use
	// for the resource. We need to get this information from the RESTMapper,
	// which uses the discovery API to determine the right GroupVersionResource.
	gvk := resource.GroupVersionKind()

	mapping, err := s.mapper.RESTMapping(gvk.GroupKind(), gvk.Version)
	if err != nil {
		return apps.ResourceActionNone, errors.Wrap(err, "get REST mapping")
	}
	var client dynamic.ResourceInterface
	if mapping.Scope.Name() == meta.RESTScopeNameRoot {
		client = s.client.Resource(mapping.Resource)
	} else {
		client = s.client.Resource(mapping.Resource).Namespace(resource.GetNamespace())
	}
	resetAppliedAnnotation := false
	if err := setAppliedAnnotation(resource); err != nil {
		log.Printf("Storing Applied Annotation failed: %v", err)
		resetAppliedAnnotation = true
	}

	// Create the resource if it doesn't exist yet.
	_, getSpan := trace.StartSpan(ctx, "Get "+resource.GetName())
	current, err := client.Get(ctx, resource.GetName(), metav1.GetOptions{})
	getSpan.End()
	if k8serrors.IsNotFound(err) {
		_, createSpan := trace.StartSpan(ctx, "Create "+resource.GetName())
		res, err := client.Create(ctx, resource, metav1.CreateOptions{})
		createSpan.End()
		if err != nil {
			return apps.ResourceActionCreate, errors.Wrap(err, "create resource")
		}
		*resource = *res
		return apps.ResourceActionCreate, nil
	} else if err != nil {
		return apps.ResourceActionNone, errors.Wrap(err, "get resource")
	}
	if err := validateOwnerRefs(current, set); err != nil {
		return apps.ResourceActionNone, errors.Wrap(err, "owner conflict")
	}

	// Get what is running, what was installed and what we want to run.
	currentRaw, err := current.MarshalJSON()
	if err != nil {
		return apps.ResourceActionNone, err
	}
	resourceRaw, err := resource.MarshalJSON()
	if err != nil {
		return apps.ResourceActionNone, err
	}
	if resetAppliedAnnotation {
		deleteAppliedAnnotation(current)
	}
	originalRaw := getAppliedAnnotation(current)

	var patchErr error
	if len(originalRaw) > 0 {
		// Try to patch it.
		var (
			patchType types.PatchType
			patch     []byte
		)
		obj, err := scheme.Scheme.New(mapping.GroupVersionKind)
		if err == nil {
			// TODO: add option to dynamically load patch meta from discovery API
			// for full kubectl compatibility.
			patchMeta, err := strategicpatch.NewPatchMetaFromStruct(obj)
			if err != nil {
				return apps.ResourceActionNone, errors.Wrap(err, "lookup patch meta")
			}
			// TODO: Make overwrite boolean configurable for full kubectl compatibility.
			patch, err = strategicpatch.CreateThreeWayMergePatch(
				originalRaw, resourceRaw, currentRaw,
				patchMeta, true,
			)
			if err != nil {
				return apps.ResourceActionNone, errors.Wrap(err, "create strategic-merge-patch")
			}
			patchType = types.StrategicMergePatchType

		} else if runtime.IsNotRegisteredError(err) {
			patch, err = jsonmergepatch.CreateThreeWayJSONMergePatch(
				originalRaw, resourceRaw, currentRaw,
				mergepatch.RequireKeyUnchanged("apiVersion"),
				mergepatch.RequireKeyUnchanged("kind"),
				mergepatch.RequireMetadataKeyUnchanged("name"),
			)
			if err != nil {
				return apps.ResourceActionNone, errors.Wrap(err, "create json-merge-patch")
			}
			patchType = types.MergePatchType
		} else {
			return apps.ResourceActionNone, errors.Wrap(err, "instantiate object")
		}
		// As the ownerReference is bumped, the patch will never be empty
		// and we cannot skip it.

		// CL https://github.com/kubernetes/kubernetes/pull/71156
		// added an option to fix a patch against a specific resourceVersion.
		// However, it isn't used anywhere in kubectl apply itself. Thus we don't do it here either.
		// Additionally the CL doesn't seem to implement valid behavior as the patch
		// retries will not update to a new resourceVersion and the failure would persist.
		_, patchSpan := trace.StartSpan(ctx, "Patch "+resource.GetName())
		res, err := client.Patch(ctx, resource.GetName(), patchType, patch, metav1.PatchOptions{})
		patchSpan.End()
		if err == nil {
			// Successfully patched.
			*resource = *res
			return apps.ResourceActionUpdate, nil
		}
		patchErr = err
	} else {
		// We don't have lastApplied state, hence try a direct Update without a
		// 3-way-merge. This can happen if the resource is too large for the
		// annotation or has been deleted.

		resource.SetResourceVersion(current.GetResourceVersion())

		_, updateSpan := trace.StartSpan(ctx, "Update "+resource.GetName())
		res, err := client.Update(ctx, resource, metav1.UpdateOptions{})
		updateSpan.End()
		if err == nil {
			// Successfully updated.
			*resource = *res
			return apps.ResourceActionUpdate, nil
		}
		patchErr = err
	}

	// If patching/updating failed, consider deleting and recreating the resource.
	if !canReplace(resource, patchErr) {
		return apps.ResourceActionUpdate, errors.Wrap(patchErr, "apply patch or update")
	}
	_, replace_span := trace.StartSpan(ctx, "Replace "+resource.GetName())
	res, err := replace(ctx, client, resource)
	replace_span.End()
	if err != nil {
		return apps.ResourceActionReplace, errors.Wrap(err, "replace")
	}
	*resource = *res
	return apps.ResourceActionReplace, nil
}

// crdAvailable checks if all versions of the given CRD are present in the
// server's discovery information. Callers must use s.Discovery.Invalidate()
// to clear the discovery cache before calling this method to check against the
// latest server state.
func (s *Synk) crdAvailable(ucrd *unstructured.Unstructured) (bool, error) {
	var crd apiextensions.CustomResourceDefinition
	if err := convert(ucrd, &crd); err != nil {
		return false, err
	}

	// Get a list of versions to check for.
	var versions []string
	for _, v := range crd.Spec.Versions {
		if v.Served {
			versions = append(versions, v.Name)
		}
	}

	for _, v := range versions {
		list, err := s.discovery.ServerResourcesForGroupVersion(crd.Spec.Group + "/" + v)
		if err != nil {
			// We'd like to detect "not found" vs network errors here. But unfortunately
			// there's no canonical error being used.
			log.Printf("ServerResourcesForGroupVersion(%s/%s) failed: %v", crd.Spec.Group, v, err)
			return false, nil
		}
		found := false
		for _, r := range list.APIResources {
			if r.Name == crd.Spec.Names.Plural {
				found = true
				break
			}
		}
		if !found {
			return false, nil
		}
	}
	return true, nil
}

var resourceSetGVR = schema.GroupVersionResource{
	Group:    "apps.cloudrobotics.com",
	Version:  "v1alpha1",
	Resource: "resourcesets",
}

func (s *Synk) createResourceSet(ctx context.Context, rs *apps.ResourceSet) error {
	rs.Kind = "ResourceSet"
	rs.APIVersion = "apps.cloudrobotics.com/v1alpha1"

	var u unstructured.Unstructured
	if err := convert(rs, &u); err != nil {
		return err
	}
	res, err := s.client.Resource(resourceSetGVR).Create(ctx, &u, metav1.CreateOptions{})
	if err != nil {
		return err
	}
	return convert(res, rs)
}

type applyResult struct {
	resource *unstructured.Unstructured
	err      error
	action   apps.ResourceAction
}

func (r *applyResult) String() string {
	return fmt.Sprintf("%s action=%s error=%s", resourceKey(r.resource), r.action, r.err)
}

type applyResults map[string]*applyResult

func (r applyResults) set(res *unstructured.Unstructured, action apps.ResourceAction, err error) {
	r[resourceKey(res)] = &applyResult{
		resource: res,
		action:   action,
		err:      err,
	}
}

func (r applyResults) failed(res *unstructured.Unstructured) bool {
	if x, ok := r[resourceKey(res)]; ok && x.err != nil {
		return true
	}
	return false
}

func (r applyResults) list() (l []*applyResult) {
	for _, res := range r {
		l = append(l, res)
	}
	sort.Slice(l, func(i, j int) bool {
		return lessUnstructured(l[i].resource, l[j].resource)
	})
	return l
}

func (s *Synk) updateResourceSetStatus(ctx context.Context, rs *apps.ResourceSet, results applyResults) error {
	type group map[schema.GroupVersionKind][]apps.ResourceStatus
	applied, failed := group{}, group{}

	for _, r := range results.list() {
		st := apps.ResourceStatus{
			Namespace:  r.resource.GetNamespace(),
			Name:       r.resource.GetName(),
			Action:     r.action,
			UID:        string(r.resource.GetUID()),
			Generation: r.resource.GetGeneration(),
		}
		if r.err != nil {
			st.Error = r.err.Error()
		}
		gvk := r.resource.GroupVersionKind()
		if r.err != nil {
			failed[gvk] = append(failed[gvk], st)
		} else {
			applied[gvk] = append(applied[gvk], st)
		}
	}
	// Attach group map as sorted status list.
	build := func(g group, list *[]apps.ResourceSetStatusGroup) {
		for gvk, res := range g {
			*list = append(*list, apps.ResourceSetStatusGroup{
				Group:   gvk.Group,
				Version: gvk.Version,
				Kind:    gvk.Kind,
				Items:   res,
			})
		}
		sort.Slice(*list, func(i, j int) bool {
			return lessResourceSetStatusGroup(&(*list)[i], &(*list)[j])
		})
	}
	build(applied, &rs.Status.Applied)
	build(failed, &rs.Status.Failed)

	rs.Status.FinishedAt = metav1.Now()
	if len(rs.Status.Failed) > 0 {
		rs.Status.Phase = apps.ResourceSetPhaseFailed
	} else {
		rs.Status.Phase = apps.ResourceSetPhaseSettled
	}

	var u unstructured.Unstructured
	if err := convert(rs, &u); err != nil {
		return err
	}
	res, err := s.client.Resource(resourceSetGVR).Update(ctx, &u, metav1.UpdateOptions{})
	if err != nil {
		return errors.Wrap(err, "update ResourceSet status")
	}
	return convert(res, rs)
}

// deleteResourceSets deletes all ResourceSets of the given name that have a lower version.
func (s *Synk) deleteResourceSets(ctx context.Context, name string, version int32) error {
	c := s.client.Resource(resourceSetGVR)

	list, err := c.List(ctx, metav1.ListOptions{})
	if err != nil {
		return errors.Wrap(err, "list existing resources")
	}
	for _, r := range list.Items {
		n, v, ok := decodeResourceSetName(r.GetName())
		if !ok || n != name || v >= version {
			continue
		}
		// TODO: should we possibly opt for foreground deletion here so
		// we only return after all dependents have been deleted as well?
		// kubectl doesn't allow to opt into foreground deletion in general but
		// here it would likely bring us closer to the apply --prune semantics.
		if err := c.Delete(ctx, r.GetName(), metav1.DeleteOptions{}); err != nil {
			return errors.Wrapf(err, "delete ResourceSet %q", r.GetName())
		}
	}
	return nil
}

// next returns the next version for the resources name.
func (s *Synk) next(ctx context.Context, name string) (version int32, err error) {
	list, err := s.client.Resource(resourceSetGVR).List(ctx, metav1.ListOptions{})
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

// Filter for helm-hooks that mark test resources. See
// https://github.com/googlecloudrobotics/core/issues/20
func isTestResource(r *unstructured.Unstructured) bool {
	hook, ok := r.GetAnnotations()["helm.sh/hook"]
	return ok && strings.HasPrefix(hook, "test-")
}

func isCustomResourceDefinition(r *unstructured.Unstructured) bool {
	return strings.HasPrefix(r.GetAPIVersion(), "apiextensions.k8s.io/") && r.GetKind() == "CustomResourceDefinition"
}

func separateCRDsFromResources(resources []*unstructured.Unstructured) (crds []*unstructured.Unstructured, regulars []*unstructured.Unstructured) {
	for _, r := range resources {
		if isCustomResourceDefinition(r) {
			crds = append(crds, r)
		} else {
			regulars = append(regulars, r)
		}
	}
	return crds, regulars
}

func filter(in []*unstructured.Unstructured, f func(*unstructured.Unstructured) bool) (out []*unstructured.Unstructured) {
	for _, r := range in {
		if f(r) {
			out = append(out, r)
		}
	}
	return out
}

func resourceSetName(s string, v int32) string {
	return fmt.Sprintf("%s.v%d", s, v)
}

var namePat = regexp.MustCompile(`^(.+)\.v([0-9]+)$`)

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
		return lessUnstructured(res[i], res[j])
	})
}

func resourceKey(r *unstructured.Unstructured) string {
	gvk := r.GroupVersionKind()
	return fmt.Sprintf("%s/%s/%s",
		gvkKey(gvk.Group, gvk.Version, gvk.Kind),
		r.GetNamespace(),
		r.GetName())
}

func gvkKey(group, version, kind string) string {
	return fmt.Sprintf("%s/%s/%s", group, version, kind)
}

// convert a resource from one type representation to another one.
func convert(from, to runtime.Object) error {
	b, err := json.Marshal(from)
	if err != nil {
		return err
	}
	return json.Unmarshal(b, &to)
}
