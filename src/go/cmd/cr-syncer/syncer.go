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
	"log"
	"net/http"
	"strconv"

	crdtypes "k8s.io/apiextensions-apiserver/pkg/apis/apiextensions/v1beta1"
	"k8s.io/apimachinery/pkg/api/errors"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/apimachinery/pkg/watch"
	"k8s.io/client-go/dynamic"
	"k8s.io/client-go/tools/cache"
	"k8s.io/client-go/util/workqueue"
)

const (
	// Annotations attached to CRDs.
	annotationStatusSubtree     = "cr-syncer.cloudrobotics.com/status-subtree"
	annotationFilterByRobotName = "cr-syncer.cloudrobotics.com/filter-by-robot-name"
	annotationSpecSource        = "cr-syncer.cloudrobotics.com/spec-source"

	// Annotations and labels attached to CRs.
	labelRobotName = "cloudrobotics.com/robot-name"
	// Annotation for remote resource version. Note that for resources in the cloud cluster,
	// this is a resource version on the robot's cluster (and vice versa).
	annotationResourceVersion = "cr-syncer.cloudrobotics.com/remote-resource-version"
)

// finalizerFor returns the finalizer for the given cluster name
func finalizerFor(clusterName string) string {
	return fmt.Sprintf("%s.synced.cr-syncer.cloudrobotics.com", clusterName)
}

func stringsContain(strs []string, v string) bool {
	for _, s := range strs {
		if s == v {
			return true
		}
	}
	return false
}

func stringsDelete(list []string, s string) (res []string) {
	for _, x := range list {
		if x != s {
			res = append(res, x)
		}
	}
	return res
}

// crSyncer synchronizes custom resources from an upstream source cluster to a
// downstream cluster.
// Updates to the status subresource in the downstream are propagated back to
// the upstream cluster.
type crSyncer struct {
	clusterName   string // Name of downstream cluster.
	crd           crdtypes.CustomResourceDefinition
	upstream      dynamic.ResourceInterface // Source of the spec.
	downstream    dynamic.ResourceInterface // Source of the status.
	labelSelector string
	subtree       string

	// Informers and the queues they feed. Upstream/downstream describes
	// the source of the change events, _not_ the direction they are heading.
	// For example, upstream{Inf,Queue} receive updates that will result in the
	// syncer taking actions against the downstream cluster.
	upstreamInf     cache.SharedIndexInformer
	downstreamInf   cache.SharedIndexInformer
	upstreamQueue   workqueue.RateLimitingInterface
	downstreamQueue workqueue.RateLimitingInterface

	done chan struct{} // Terminates all background processes.
}

func newCRSyncer(
	crd crdtypes.CustomResourceDefinition,
	local, remote dynamic.Interface,
	robotName string,
) (*crSyncer, error) {
	var (
		annotations        = crd.ObjectMeta.Annotations
		filterByRobotValue = annotations[annotationFilterByRobotName]
		filterByRobot      = false
	)
	if filterByRobotValue != "" {
		if v, err := strconv.ParseBool(filterByRobotValue); err != nil {
			log.Printf("Value for %s  must be boolean on %s, got %q",
				annotationFilterByRobotName, crd.ObjectMeta.Name, filterByRobotValue)
		} else {
			filterByRobot = v
		}
	}
	gvr := schema.GroupVersionResource{
		Group:    crd.Spec.Group,
		Version:  crd.Spec.Version,
		Resource: crd.Spec.Names.Plural,
	}
	s := &crSyncer{
		crd:             crd,
		subtree:         annotations[annotationStatusSubtree],
		upstream:        remote.Resource(gvr).Namespace("default"),
		downstream:      local.Resource(gvr).Namespace("default"),
		upstreamQueue:   workqueue.NewNamedRateLimitingQueue(workqueue.DefaultControllerRateLimiter(), "upstream"),
		downstreamQueue: workqueue.NewNamedRateLimitingQueue(workqueue.DefaultControllerRateLimiter(), "downstream"),
		done:            make(chan struct{}),
	}
	switch src := annotations[annotationSpecSource]; src {
	case "robot":
		s.clusterName = "cloud"
		// Swap upstream and downstream if the robot is the spec source.
		s.upstream, s.downstream = s.downstream, s.upstream
	case "cloud":
		s.clusterName = fmt.Sprintf("robot-%s", robotName)
	default:
		return nil, fmt.Errorf("unknown spec source %q", src)
	}
	if filterByRobot {
		if robotName != "" {
			s.labelSelector = labelRobotName + "=" + robotName
		} else {
			// TODO(fabxc): should this return an error instead?
			log.Printf("%s requested to filter by robot-name, but no robot-name was given to cr-syncer", crd.ObjectMeta.Name)
		}
	}

	newInformer := func(client dynamic.ResourceInterface) cache.SharedIndexInformer {
		return cache.NewSharedIndexInformer(
			&cache.ListWatch{
				ListFunc: func(options metav1.ListOptions) (runtime.Object, error) {
					options.LabelSelector = s.labelSelector
					return client.List(options)
				},
				WatchFunc: func(options metav1.ListOptions) (watch.Interface, error) {
					options.LabelSelector = s.labelSelector
					return client.Watch(options)
				},
			},
			&unstructured.Unstructured{},
			resyncPeriod,
			nil,
		)
	}
	s.upstreamInf = newInformer(s.upstream)
	s.downstreamInf = newInformer(s.downstream)

	return s, nil
}

func (s *crSyncer) startInformers() error {
	go s.upstreamInf.Run(s.done)
	go s.downstreamInf.Run(s.done)

	if ok := cache.WaitForCacheSync(s.done, s.upstreamInf.HasSynced); !ok {
		return fmt.Errorf("stopped while syncing upstream informer for %s", s.crd.GetName())
	}
	if ok := cache.WaitForCacheSync(s.done, s.downstreamInf.HasSynced); !ok {
		return fmt.Errorf("stopped while syncing downstream informer for %s", s.crd.GetName())
	}
	s.setupInformerHandlers(s.upstreamInf, s.upstreamQueue, "upstream")
	s.setupInformerHandlers(s.downstreamInf, s.downstreamQueue, "downstream")

	return nil
}

func (s *crSyncer) setupInformerHandlers(
	inf cache.SharedIndexInformer,
	queue workqueue.RateLimitingInterface,
	direction string,
) {
	receive := func(obj interface{}, action string) {
		u := obj.(*unstructured.Unstructured)
		log.Printf("Got %s event from %s for %s %s@v%s",
			action, direction, u.GetKind(), u.GetName(), u.GetResourceVersion())
		if key, ok := keyFunc(obj); ok {
			queue.AddRateLimited(key)
		}
	}
	inf.AddEventHandler(cache.ResourceEventHandlerFuncs{
		AddFunc: func(obj interface{}) {
			receive(obj, "add")
		},
		UpdateFunc: func(_, obj interface{}) {
			receive(obj, "update")
		},
		DeleteFunc: func(obj interface{}) {
			receive(obj, "delete")
		},
	})
}

func (s *crSyncer) processNextWorkItem(
	q workqueue.RateLimitingInterface,
	syncf func(string) error,
	qName string,
) bool {
	key, quit := q.Get()
	if quit {
		return false
	}
	defer q.Done(key)

	err := syncf(key.(string))
	if err == nil {
		q.Forget(key)
		return true
	}
	// Synchronization failed, retry later.
	log.Printf("Syncing key %q from queue %q failed: %s", key, qName, err)
	q.AddRateLimited(key)

	return true
}

func (s *crSyncer) run() {
	defer s.upstreamQueue.ShutDown()
	defer s.downstreamQueue.ShutDown()

	log.Printf("Starting syncer for %s", s.crd.GetName())

	// Start informers that will populate their associated workqueue.
	if err := s.startInformers(); err != nil {
		log.Printf("Starting informers for %s failed: %s", s.crd.GetName(), err)
		return
	}
	// Process the upstream and downstream work queues.
	go func() {
		for s.processNextWorkItem(s.upstreamQueue, s.syncUpstream, "upstream") {
		}
	}()
	go func() {
		for s.processNextWorkItem(s.downstreamQueue, s.syncDownstream, "downstream") {
		}
	}()
	<-s.done
}

func (s *crSyncer) stop() {
	log.Printf("Stopping syncer for %s", s.crd.GetName())
	close(s.done)
}

// syncDownstream reconciles state after receiving change events from the downstream cluster.
// It synchronizes the status from the downstream to the upstream cluster, removes
// fnializers from upstream and downstream if both resources can be safely deleted, and
// deletes orphaned downstream resources.
func (s *crSyncer) syncDownstream(key string) error {
	var (
		finalizer           = finalizerFor(s.clusterName)
		statusIsSubresource = s.crd.Spec.Subresources != nil && s.crd.Spec.Subresources.Status != nil
	)
	// Get a recent version of the upstream spec.
	srcObj, exists, err := s.downstreamInf.GetIndexer().GetByKey(key)
	if err != nil {
		return fmt.Errorf("failed to retrieve resource for key %s: %s", key, err)
	}
	if !exists {
		return nil
	}
	src := srcObj.(*unstructured.Unstructured).DeepCopy()

	dstObj, exists, err := s.upstreamInf.GetIndexer().GetByKey(key)
	if err != nil {
		return fmt.Errorf("failed to retrieve resource for key %s: %s", key, err)
	}
	// Upstream resource no longer exists. Remove the finalizer from the downstream
	// resource and delete it.
	if !exists {
		src.SetFinalizers(stringsDelete(src.GetFinalizers(), finalizer))
		if _, err := s.downstream.Update(src, metav1.UpdateOptions{}); err != nil {
			if isNotFoundError(err) {
				return nil
			}
			return fmt.Errorf("remove finalizer: %s", err)
		}
		if src.GetDeletionTimestamp() != nil {
			return nil // Already deleted.
		}
		if err := s.downstream.Delete(src.GetName(), nil); err != nil {
			if isNotFoundError(err) {
				return nil
			}
			return fmt.Errorf("delete resource: %s", err)
		}
		return nil
	}
	dst := dstObj.(*unstructured.Unstructured).DeepCopy()

	// Copy full status or subtree from src to dst.
	if s.subtree == "" {
		dst.Object["status"] = src.Object["status"]
	} else if src.Object["status"] != nil {
		srcStatus, ok := src.Object["status"].(map[string]interface{})
		if !ok {
			return fmt.Errorf("Expected status of %s in downstream cluster to be a dict", src.GetName())
		}
		if dst.Object["status"] == nil {
			dst.Object["status"] = make(map[string]interface{})
		}
		dstStatus, ok := dst.Object["status"].(map[string]interface{})
		if !ok {
			return fmt.Errorf("Expected status of %s in upstream cluster to be a dict", src.GetName())
		}
		if srcStatus[s.subtree] != nil {
			dstStatus[s.subtree] = srcStatus[s.subtree]
		} else {
			delete(dstStatus, s.subtree)
		}
	}
	setAnnotation(dst, annotationResourceVersion, src.GetResourceVersion())

	// We need to make a dedicated UpdateStatus call if the status is defined
	// as an explicit subresource of the CRD.
	if statusIsSubresource {
		// Status must not be null/nil.
		if dst.Object["status"] == nil {
			dst.Object["status"] = struct{}{}
		}
		updated, err := s.upstream.UpdateStatus(dst, metav1.UpdateOptions{})
		if err != nil {
			return newAPIErrorf(dst, "update status failed: %s", err)
		}
		dst = updated
	} else {
		updated, err := s.upstream.Update(dst, metav1.UpdateOptions{})
		if err != nil {
			return newAPIErrorf(dst, "update failed: %s", err)
		}
		dst = updated
	}
	log.Printf("Copied %s %s status@v%s to upstream@v%s",
		src.GetKind(), src.GetName(), src.GetResourceVersion(), src.GetResourceVersion())

	if src.GetDeletionTimestamp() == nil {
		return nil
	}
	// Downstream resource is pending for deletion. Once all downstream finalizers but our
	// own are removed, drop it from upstream and downstream in order. The order
	// ensures that the upstream resource is not left with a dangling finalizer.
	if len(src.GetFinalizers()) == 1 && stringsContain(src.GetFinalizers(), finalizer) {
		dst.SetFinalizers(stringsDelete(src.GetFinalizers(), finalizer))
		src.SetFinalizers(nil)

		if _, err := s.upstream.Update(dst, metav1.UpdateOptions{}); err != nil {
			return newAPIErrorf(dst, "upstream update failed: %s", err)
		}
		// If we crash before reaching this point, the orphan cleanup above
		// will ensure the downstream resource is deleted.
		if _, err := s.downstream.Update(src, metav1.UpdateOptions{}); err != nil {
			return newAPIErrorf(src, "downstream update failed: %s", err)
		}
	}
	return nil
}

// syncUpstream reconciles the state after receiving a change event from upstream.
// It synchronizes the spec changes from usptream to the downstream cluster and propagates
// deletions.
// It attaches a finalizer to the upstream and downstream resource in order to
// ensure that downstream resources are properly cleaned up before the upstream resource
// can complete deletion.
func (s *crSyncer) syncUpstream(key string) error {
	// Create-or-update function.
	upsert := func(o *unstructured.Unstructured) (*unstructured.Unstructured, error) {
		return s.downstream.Update(o, metav1.UpdateOptions{})
	}
	// Get a recent version of the upstream spec.
	srcObj, exists, err := s.upstreamInf.GetIndexer().GetByKey(key)
	if err != nil {
		return fmt.Errorf("failed to retrieve resource for key %s: %s", key, err)
	}
	if !exists {
		return nil
	}
	src := srcObj.(*unstructured.Unstructured).DeepCopy()

	dstObj, exists, err := s.downstreamInf.GetIndexer().GetByKey(key)
	if err != nil {
		return fmt.Errorf("failed to retrieve resource for key %s: %s", key, err)
	}
	dst := &unstructured.Unstructured{}
	if exists {
		dst = dstObj.(*unstructured.Unstructured).DeepCopy()
	} else {
		// Create object and set base fields.
		upsert = func(o *unstructured.Unstructured) (*unstructured.Unstructured, error) {
			o.SetGroupVersionKind(src.GroupVersionKind())
			o.SetNamespace(src.GetNamespace())
			o.SetName(src.GetName())
			// Copy upstream status on initial creation.
			o.Object["status"] = src.Object["status"]

			return s.downstream.Create(o, metav1.CreateOptions{})
		}
	}

	finalizer := finalizerFor(s.clusterName)

	if !stringsContain(dst.GetFinalizers(), finalizer) {
		dst.SetFinalizers(append(dst.GetFinalizers(), finalizer))
	}
	dst.SetLabels(src.GetLabels())
	dst.SetAnnotations(src.GetAnnotations())
	deleteAnnotation(dst, annotationResourceVersion)

	dst.Object["spec"] = src.Object["spec"]

	// Ensure that our finalizer is set in the upstream resource before the downstream
	// resource is created.
	if src.GetDeletionTimestamp() == nil && !stringsContain(src.GetFinalizers(), finalizer) {
		src.SetFinalizers(append(src.GetFinalizers(), finalizer))

		if _, err := s.upstream.Update(src, metav1.UpdateOptions{}); err != nil {
			return newAPIErrorf(src, "setting upstream finalizer failed: %s", err)
		}
	}
	// Now we can safely create/update the resource downstream and the upstream
	// resource will not go away until we've deleted it again.
	updated, err := upsert(dst)
	if err != nil {
		return newAPIErrorf(dst, "failed to upsert downstream: %s", err)
	}
	dst = updated

	// Mark downstream resource deleted if upstream resource is being deleted.
	if src.GetDeletionTimestamp() != nil && dst.GetDeletionTimestamp() == nil {
		if err := s.downstream.Delete(dst.GetName(), nil); err != nil {
			return newAPIErrorf(dst, "downstream delete failed: %s", err)
		}
		return nil
	}
	return nil
}

func isNotFoundError(err error) bool {
	status, ok := err.(*errors.StatusError)
	return ok && status.ErrStatus.Code == http.StatusNotFound
}

type apiError struct {
	o   *unstructured.Unstructured
	msg string
}

func (e apiError) Error() string {
	return fmt.Sprintf("%s %s/%s @ %s: %s", e.o.GetKind(), e.o.GetNamespace(), e.o.GetName(), e.o.GetResourceVersion(), e.msg)
}

func newAPIErrorf(o *unstructured.Unstructured, format string, args ...interface{}) apiError {
	return apiError{o: o, msg: fmt.Sprintf(format, args...)}
}

// keyFunc extracts a key of the form [<namespace>/]<name> from a resource
// which is used to access the informer's store and index.
func keyFunc(obj interface{}) (string, bool) {
	k, err := cache.DeletionHandlingMetaNamespaceKeyFunc(obj)
	if err != nil {
		log.Printf("deriving key failed: %s", err)
		return k, false
	}
	return k, true
}
