// Copyright 2026 The Cloud Robotics Authors
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
	"context"
	"encoding/json"
	"fmt"
	"math/rand"
	"sync/atomic"
	"testing"
	"time"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	apiextensions "k8s.io/apiextensions-apiserver/pkg/apis/apiextensions/v1"
	k8serrors "k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/api/meta/testrestmapper"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/client-go/discovery"
	"k8s.io/client-go/dynamic"
	dynamicfake "k8s.io/client-go/dynamic/fake"
	"k8s.io/client-go/kubernetes/scheme"
	k8stest "k8s.io/client-go/testing"
)

type benchDiscoveryClient struct {
	discovery.CachedDiscoveryInterface
	resources      map[string]*metav1.APIResourceList
	latency        time.Duration
	discoveryCalls atomic.Int64
}

func (d *benchDiscoveryClient) Invalidate() {}

func (d *benchDiscoveryClient) ServerGroupsAndResources() ([]*metav1.APIGroup, []*metav1.APIResourceList, error) {
	d.discoveryCalls.Add(1)
	if d.latency > 0 {
		time.Sleep(d.latency)
	}
	var list []*metav1.APIResourceList
	for _, l := range d.resources {
		list = append(list, l)
	}
	return nil, list, nil
}

func (d *benchDiscoveryClient) ServerResourcesForGroupVersion(gv string) (*metav1.APIResourceList, error) {
	d.discoveryCalls.Add(1)
	if d.latency > 0 {
		time.Sleep(d.latency)
	}
	if list, ok := d.resources[gv]; ok {
		return list, nil
	}
	return nil, k8serrors.NewNotFound(schema.GroupResource{}, gv)
}

type latencyDynamicClient struct {
	delegate dynamic.Interface
	latency  time.Duration
	apiCalls *atomic.Int64
}

func (c *latencyDynamicClient) Resource(resource schema.GroupVersionResource) dynamic.NamespaceableResourceInterface {
	res := c.delegate.Resource(resource)
	return &latencyNamespaceableResource{
		nsDelegate: res,
		latencyResource: latencyResource{
			ResourceInterface: res,
			latency:           c.latency,
			apiCalls:          c.apiCalls,
		},
	}
}

type latencyNamespaceableResource struct {
	nsDelegate dynamic.NamespaceableResourceInterface
	latencyResource
}

func (r *latencyNamespaceableResource) Namespace(ns string) dynamic.ResourceInterface {
	return &latencyResource{
		ResourceInterface: r.nsDelegate.Namespace(ns),
		latency:           r.latency,
		apiCalls:          r.apiCalls,
	}
}

type latencyResource struct {
	dynamic.ResourceInterface
	latency  time.Duration
	apiCalls *atomic.Int64
}

func (r *latencyResource) roundTrip() {
	r.apiCalls.Add(1)
	if r.latency > 0 {
		time.Sleep(r.latency)
	}
}

func (r *latencyResource) Create(ctx context.Context, obj *unstructured.Unstructured, options metav1.CreateOptions, subresources ...string) (*unstructured.Unstructured, error) {
	r.roundTrip()
	return r.ResourceInterface.Create(ctx, obj, options, subresources...)
}

func (r *latencyResource) Update(ctx context.Context, obj *unstructured.Unstructured, options metav1.UpdateOptions, subresources ...string) (*unstructured.Unstructured, error) {
	r.roundTrip()
	return r.ResourceInterface.Update(ctx, obj, options, subresources...)
}

func (r *latencyResource) Delete(ctx context.Context, name string, options metav1.DeleteOptions, subresources ...string) error {
	r.roundTrip()
	return r.ResourceInterface.Delete(ctx, name, options, subresources...)
}

func (r *latencyResource) DeleteCollection(ctx context.Context, options metav1.DeleteOptions, listOptions metav1.ListOptions) error {
	r.roundTrip()
	return r.ResourceInterface.DeleteCollection(ctx, options, listOptions)
}

func (r *latencyResource) Get(ctx context.Context, name string, options metav1.GetOptions, subresources ...string) (*unstructured.Unstructured, error) {
	r.roundTrip()
	return r.ResourceInterface.Get(ctx, name, options, subresources...)
}

func (r *latencyResource) List(ctx context.Context, opts metav1.ListOptions) (*unstructured.UnstructuredList, error) {
	r.roundTrip()
	return r.ResourceInterface.List(ctx, opts)
}

func (r *latencyResource) Patch(ctx context.Context, name string, pt types.PatchType, data []byte, options metav1.PatchOptions, subresources ...string) (*unstructured.Unstructured, error) {
	r.roundTrip()
	return r.ResourceInterface.Patch(ctx, name, pt, data, options, subresources...)
}

type benchFixture struct {
	synk      *Synk
	fake      *k8stest.Fake
	discovery *benchDiscoveryClient
	apiCalls  atomic.Int64
}

func newBenchFixture(latency time.Duration) *benchFixture {
	disc := &benchDiscoveryClient{
		latency: latency,
		resources: map[string]*metav1.APIResourceList{
			"v1": {
				GroupVersion: "v1",
				APIResources: []metav1.APIResource{
					{Name: "namespaces", Kind: "Namespace", Namespaced: false},
					{Name: "serviceaccounts", Kind: "ServiceAccount", Namespaced: true},
					{Name: "secrets", Kind: "Secret", Namespaced: true},
					{Name: "configmaps", Kind: "ConfigMap", Namespaced: true},
					{Name: "services", Kind: "Service", Namespaced: true},
					{Name: "pods", Kind: "Pod", Namespaced: true},
				},
			},
			"rbac.authorization.k8s.io/v1": {
				GroupVersion: "rbac.authorization.k8s.io/v1",
				APIResources: []metav1.APIResource{
					{Name: "clusterroles", Kind: "ClusterRole", Namespaced: false},
					{Name: "clusterrolebindings", Kind: "ClusterRoleBinding", Namespaced: false},
					{Name: "roles", Kind: "Role", Namespaced: true},
					{Name: "rolebindings", Kind: "RoleBinding", Namespaced: true},
				},
			},
			"apps/v1": {
				GroupVersion: "apps/v1",
				APIResources: []metav1.APIResource{
					{Name: "deployments", Kind: "Deployment", Namespaced: true},
				},
			},
			"apps.cloudrobotics.com/v1alpha1": {
				GroupVersion: "apps.cloudrobotics.com/v1alpha1",
				APIResources: []metav1.APIResource{
					{Name: "resourcesets", Kind: "ResourceSet", Namespaced: false},
				},
			},
		},
	}

	sc := runtime.NewScheme()
	scheme.AddToScheme(sc)
	apps.AddToScheme(sc)
	apiextensions.AddToScheme(sc)

	bf := &benchFixture{
		discovery: disc,
	}

	client := dynamicfake.NewSimpleDynamicClient(sc)
	wrappedClient := &latencyDynamicClient{
		delegate: client,
		latency:  latency,
		apiCalls: &bf.apiCalls,
	}
	s := New(wrappedClient, disc)
	s.mapper = testrestmapper.TestOnlyStaticRESTMapper(sc)
	s.resetMapper = func() {}

	bf.synk = s
	bf.fake = &client.Fake

	// client-go's default ObjectTracker cannot apply StrategicMergePatchType
	// directly to *unstructured.Unstructured objects, so handle patch on the
	// tracker's unstructured state.
	bf.fake.PrependReactor("patch", "*", func(action k8stest.Action) (bool, runtime.Object, error) {
		pa := action.(k8stest.PatchAction)
		obj, err := client.Tracker().Get(pa.GetResource(), pa.GetNamespace(), pa.GetName())
		if err != nil {
			return true, nil, err
		}
		u := obj.(*unstructured.Unstructured).DeepCopy()
		var patchObj map[string]interface{}
		if err := json.Unmarshal(pa.GetPatch(), &patchObj); err != nil {
			return true, nil, err
		}
		if metaPatch, ok := patchObj["metadata"].(map[string]interface{}); ok {
			if anns, ok := metaPatch["annotations"].(map[string]interface{}); ok {
				existingAnns := u.GetAnnotations()
				if existingAnns == nil {
					existingAnns = map[string]string{}
				}
				for k, v := range anns {
					if s, ok := v.(string); ok {
						existingAnns[k] = s
					}
				}
				u.SetAnnotations(existingAnns)
			}
		}
		if err := client.Tracker().Update(pa.GetResource(), u, pa.GetNamespace()); err != nil {
			return true, nil, err
		}
		return true, u, nil
	})

	return bf
}

func generateBenchmarkResources(n int) []*unstructured.Unstructured {
	resources := make([]*unstructured.Unstructured, 0, n)
	for i := 0; i < n; i++ {
		var u *unstructured.Unstructured
		switch i % 10 {
		case 0:
			u = newUnstructured("v1", "Namespace", "", fmt.Sprintf("ns-%d", i))
		case 1:
			u = newUnstructured("v1", "ServiceAccount", "default", fmt.Sprintf("sa-%d", i))
		case 2:
			u = newUnstructured("v1", "Secret", "default", fmt.Sprintf("secret-%d", i))
			u.Object["stringData"] = map[string]interface{}{"token": "secret-value"}
		case 3:
			u = newUnstructured("rbac.authorization.k8s.io/v1", "ClusterRole", "", fmt.Sprintf("cr-%d", i))
		case 4:
			u = newUnstructured("rbac.authorization.k8s.io/v1", "ClusterRoleBinding", "", fmt.Sprintf("crb-%d", i))
		case 5:
			u = newUnstructured("rbac.authorization.k8s.io/v1", "Role", "default", fmt.Sprintf("role-%d", i))
		case 6:
			u = newUnstructured("rbac.authorization.k8s.io/v1", "RoleBinding", "default", fmt.Sprintf("rb-%d", i))
		case 7:
			u = newUnstructured("v1", "ConfigMap", "default", fmt.Sprintf("cm-%d", i))
			u.Object["data"] = map[string]interface{}{
				"config.yaml": "key1: val1\nkey2: val2\n",
			}
		case 8:
			u = newUnstructured("v1", "Service", "default", fmt.Sprintf("svc-%d", i))
			u.Object["spec"] = map[string]interface{}{
				"selector": map[string]interface{}{"app": fmt.Sprintf("app-%d", i)},
				"ports": []interface{}{
					map[string]interface{}{"port": int64(80), "targetPort": int64(8080)},
				},
			}
		case 9:
			u = newUnstructured("apps/v1", "Deployment", "default", fmt.Sprintf("deploy-%d", i))
			u.Object["spec"] = map[string]interface{}{
				"replicas": int64(2),
				"selector": map[string]interface{}{
					"matchLabels": map[string]interface{}{"app": fmt.Sprintf("app-%d", i)},
				},
				"template": map[string]interface{}{
					"metadata": map[string]interface{}{
						"labels": map[string]interface{}{"app": fmt.Sprintf("app-%d", i)},
					},
					"spec": map[string]interface{}{
						"containers": []interface{}{
							map[string]interface{}{
								"name":  "main",
								"image": "gcr.io/cloud-robotics/app:v1",
							},
						},
					},
				},
			}
		}
		resources = append(resources, u)
	}

	// Deterministic shuffle so input is unsorted.
	rng := rand.New(rand.NewSource(42))
	rng.Shuffle(len(resources), func(i, j int) {
		resources[i], resources[j] = resources[j], resources[i]
	})
	return resources
}

func BenchmarkSortResources(b *testing.B) {
	base := generateBenchmarkResources(200)
	input := make([]*unstructured.Unstructured, len(base))

	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		copy(input, base)
		sortResources(input)
	}
}

func BenchmarkApply_Create(b *testing.B) {
	resources := generateBenchmarkResources(100)
	opts := &ApplyOptions{Namespace: "default"}

	var totalAPICalls, totalDiscoveryCalls int64
	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		b.StopTimer()
		bf := newBenchFixture(0)
		b.StartTimer()

		if _, err := bf.synk.Apply(b.Context(), "bench-chart", opts, resources...); err != nil {
			b.Fatalf("Apply failed: %v", err)
		}
		totalAPICalls += bf.apiCalls.Load()
		totalDiscoveryCalls += bf.discovery.discoveryCalls.Load()
	}
	b.ReportMetric(float64(totalAPICalls)/float64(b.N), "api-calls/op")
	b.ReportMetric(float64(totalDiscoveryCalls)/float64(b.N), "discovery-calls/op")
}

func BenchmarkApply_Update(b *testing.B) {
	resources := generateBenchmarkResources(100)
	opts := &ApplyOptions{Namespace: "default"}
	bf := newBenchFixture(0)

	// Initial apply so all resources exist in the fake cluster with last-applied annotations.
	if _, err := bf.synk.Apply(b.Context(), "bench-chart", opts, resources...); err != nil {
		b.Fatalf("initial Apply failed: %v", err)
	}
	bf.apiCalls.Store(0)
	bf.discovery.discoveryCalls.Store(0)
	bf.fake.ClearActions()

	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		if _, err := bf.synk.Apply(b.Context(), "bench-chart", opts, resources...); err != nil {
			b.Fatalf("update Apply failed: %v", err)
		}
		bf.fake.ClearActions()
	}
	b.ReportMetric(float64(bf.apiCalls.Load())/float64(b.N), "api-calls/op")
	b.ReportMetric(float64(bf.discovery.discoveryCalls.Load())/float64(b.N), "discovery-calls/op")
}

func BenchmarkApply_SimulatedLatency(b *testing.B) {
	resources := generateBenchmarkResources(50)
	opts := &ApplyOptions{Namespace: "default"}
	bf := newBenchFixture(500 * time.Microsecond)

	if _, err := bf.synk.Apply(b.Context(), "bench-chart", opts, resources...); err != nil {
		b.Fatalf("initial Apply failed: %v", err)
	}
	bf.apiCalls.Store(0)
	bf.discovery.discoveryCalls.Store(0)
	bf.fake.ClearActions()

	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		if _, err := bf.synk.Apply(b.Context(), "bench-chart", opts, resources...); err != nil {
			b.Fatalf("Apply with simulated latency failed: %v", err)
		}
		bf.fake.ClearActions()
	}
	b.ReportMetric(float64(bf.apiCalls.Load())/float64(b.N), "api-calls/op")
	b.ReportMetric(float64(bf.discovery.discoveryCalls.Load())/float64(b.N), "discovery-calls/op")
}
