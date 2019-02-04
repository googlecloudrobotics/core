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
	"fmt"
	"log"
	"net/http"
	"strconv"
	"time"

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
	// Annotation that indicates that a spec should be deleted when it no
	// longer exists in the upstream cluster.
	annotationOwnedByUpstream = "cr-syncer.cloudrobotics.com/owned-by-upstream"
	// Annotation for remote resource version. Note that for resources in the cloud cluster,
	// this is a resource version on the robot's cluster (and vice versa).
	annotationResourceVersion = "cr-syncer.cloudrobotics.com/remote-resource-version"
)

// crSyncer synchronizes custom resources from an upstream source cluster to a
// downstream cluster.
// Updates to the status subresource in the downstream are propagated back to
// the upstream cluster.
type crSyncer struct {
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
		// Swap upstream and downstream if the robot is the spec source.
		s.upstream, s.downstream = s.downstream, s.upstream
	case "cloud":
		// Nothing to do.
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
		for s.processNextWorkItem(s.upstreamQueue, s.copySpec, "upstream") {
		}
	}()
	go func() {
		for s.processNextWorkItem(s.downstreamQueue, s.copyStatus, "downstream") {
		}
	}()
	// Periodically reconcile deleted resources.
	syncDeletes := time.NewTicker(resyncPeriod)
	defer syncDeletes.Stop()

	for {
		select {
		case <-syncDeletes.C:
			if err := s.syncDeletes(); err != nil {
				log.Printf("Syncing deletes failed: %s", err)
			}
		case <-s.done:
			return
		}
	}
}

func (s *crSyncer) stop() {
	log.Printf("Stopping syncer for %s", s.crd.GetName())
	close(s.done)
}

func (s *crSyncer) copyStatus(key string) error {
	// Get a recent version of the downstream status.
	obj, exists, err := s.downstreamInf.GetIndexer().GetByKey(key)
	if err != nil {
		return fmt.Errorf("failed to retrieve resource for key %s: %s", key, err)
	}
	if !exists {
		// Object was dropped from the cache since we enqueued it or it
		// was an actual deletion event.
		// We ignore it since the deletion was triggered by the upstream
		// resource and we no longer need to propagate status information.
		return nil
	}
	source := obj.(*unstructured.Unstructured)

	updated, err := tryCopyStatus(s.upstream, s.subtree, source)
	if err != nil {
		return fmt.Errorf("failed to update %s %s from downstream@v%s: %s",
			source.GetKind(), source.GetName(), source.GetResourceVersion(), err)
	} else if updated == nil {
		log.Printf("Copied %s %s status@v%s to upstream@UNKNOWN",
			source.GetKind(), source.GetName(), source.GetResourceVersion())
	} else {
		log.Printf("Copied %s %s status@v%s to upstream@v%s",
			source.GetKind(), source.GetName(), source.GetResourceVersion(), updated.GetResourceVersion())
	}
	return nil
}

func (s *crSyncer) copySpec(key string) error {
	// Get a recent version of the upstream spec.
	obj, exists, err := s.upstreamInf.GetIndexer().GetByKey(key)
	if err != nil {
		return fmt.Errorf("failed to retrieve resource for key %s: %s", key, err)
	}
	if !exists {
		// If the object was actually deleted, remove it from the downstream.
		_, name, err := cache.SplitMetaNamespaceKey(key)
		if err != nil {
			return err
		}
		if err := s.downstream.Delete(name, nil); err != nil {
			status, ok := err.(*errors.StatusError)
			if ok && status.ErrStatus.Code == http.StatusNotFound {
				// An earlier run of the syncer that didn't persist its
				// progress might already have deleted the object.
				return nil
			}
			return fmt.Errorf("Deletion failed: %v", err)
		}
		return nil
	}
	source := obj.(*unstructured.Unstructured)

	if err := createOrReplaceSpec(s.downstream, source); err != nil {
		return fmt.Errorf("failed to update downstream spec for %s %s@v%s: %s",
			source.GetKind(), source.GetName(), source.GetResourceVersion(), err)
	}
	log.Printf("Updated downstream spec for %s %s", source.GetKind(), source.GetName())
	return nil
}

// syncDeletes synchronizes deleted resources between both clusters.
//
// We generally trust that we receive all update, add, and delete events
// and apply them in our loop in run(). However, if the syncer is not running
// during a delete event, there is a chance it may be lost forever.
// This can result in resources that were created downstream to stick around
// forever while the upstream resource is already deleted.
// Kubernetes generally solves this through ownerReferences, which a garbage
// collection controller follow and then deletes orphaned resources. This
// does not work when synchronizing across clusters. Thus we've to implement
// dedicated logic here.
func (s *crSyncer) syncDeletes() error {
	kind := s.crd.Spec.Names.Kind

	for _, obj := range s.downstreamInf.GetIndexer().List() {
		o, ok := obj.(*unstructured.Unstructured)
		if !ok {
			log.Printf("Converting to unstructured failed, got %T", obj)
			continue
		}
		// Abort early if the resource is not explicitly owned by the
		// upstream cluster.
		ownedByUpstream := o.GetAnnotations()[annotationOwnedByUpstream]
		if ownedByUpstream != "" {
			v, err := strconv.ParseBool(ownedByUpstream)
			if err != nil {
				log.Printf("Value for %s must be boolean on %s, got %q",
					annotationOwnedByUpstream, kind, ownedByUpstream)
				continue
			}
			if !v {
				continue
			}
		} else {
			continue
		}

		key, ok := keyFunc(o)
		if !ok {
			continue
		}
		// If the resource no longer exists upstream, we can delete it
		// downstream.
		_, exists, err := s.upstreamInf.GetIndexer().GetByKey(key)
		if err != nil {
			log.Printf("Unexpected error when reconciling deletions for %s %s", kind, key)
			continue
		}
		if exists {
			continue
		}
		if err := s.downstream.Delete(o.GetName(), nil); err != nil {
			log.Printf("Deleting resource %s %s failed: %s", kind, key, err)
		} else {
			log.Printf("Deleted obsolete resource %s %s", kind, key)
		}
	}
	return nil
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
