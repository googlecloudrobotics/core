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
	"context"
	"fmt"
	"log/slog"
	"net/http"
	"os"
	"strconv"
	"time"

	"github.com/googlecloudrobotics/ilog"
	"github.com/pkg/errors"
	"go.opencensus.io/stats"
	"go.opencensus.io/stats/view"
	"go.opencensus.io/tag"
	crdtypes "k8s.io/apiextensions-apiserver/pkg/apis/apiextensions/v1"
	k8serrors "k8s.io/apimachinery/pkg/api/errors"
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
	// Annotation for remote resource version. Note that for resources in
	// the cloud cluster, this is a resource version on the robot's cluster
	// (and vice versa). This will only be set when the status subresource
	// is disabled, otherwise the status and annotation cannot be updated
	// in a single request.
	annotationResourceVersion = "cr-syncer.cloudrobotics.com/remote-resource-version"

	cloudClusterName = "cloud"
)

var (
	mSyncs = stats.Int64(
		"cr-syncer.cloudrobotics.com/syncs",
		"Synchronizations triggered by resource events",
		stats.UnitDimensionless,
	)
	mSyncErrors = stats.Int64(
		"cr-syncer.cloudrobotics.com/sync_errors",
		"Synchronization errors on resource events",
		stats.UnitDimensionless,
	)
	tagEventSource = mustNewTagKey("event_source")
	tagResource    = mustNewTagKey("resource")
)

func init() {
	if err := view.Register(
		&view.View{
			Name:        "cr-syncer.cloudrobotics.com/syncs_total",
			Description: "Total number of synchronizations triggered resource events",
			Measure:     mSyncs,
			TagKeys:     []tag.Key{tagEventSource, tagResource},
			Aggregation: view.Count(),
		},
		&view.View{
			Name:        "cr-syncer.cloudrobotics.com/sync_errors_total",
			Description: "Total number of synchronizations errors on resource events",
			Measure:     mSyncErrors,
			TagKeys:     []tag.Key{tagEventSource, tagResource},
			Aggregation: view.Count(),
		},
	); err != nil {
		panic(err)
	}
}

// removeFinalizer removes the cr-syncer finalizer for this robot. Finalizers
// for offline robots have to be removed manually (eg with `kubectl edit`).
// TODO(rodrigoq): remove after migration
func removeFinalizer(ctx context.Context, client dynamic.ResourceInterface, obj *unstructured.Unstructured, clusterName string) {
	update := false
	thisFinalizer := fmt.Sprintf("%s.synced.cr-syncer.cloudrobotics.com", clusterName)
	finalizers := []string{}
	for _, x := range obj.GetFinalizers() {
		if x == thisFinalizer {
			update = true
		} else {
			finalizers = append(finalizers, x)
		}
	}
	if !update {
		return
	}
	obj.SetFinalizers(finalizers)
	if _, err := client.Update(ctx, obj, metav1.UpdateOptions{}); err != nil {
		if isNotFoundError(err) {
			return
		}
		slog.Error("failed to remove finalizers", ilog.Err(err))
	}
}

// crSyncer synchronizes custom resources from an upstream source cluster to a
// downstream cluster.
// Updates to the status subresource in the downstream are propagated back to
// the upstream cluster.
type crSyncer struct {
	ctx           context.Context
	clusterName   string // Name of downstream cluster.
	crd           crdtypes.CustomResourceDefinition
	upstream      dynamic.ResourceInterface // Source of the spec.
	downstream    dynamic.ResourceInterface // Source of the status.
	labelSelector string
	subtree       string
	versionIx     int

	// Informers and the queues they feed. Upstream/downstream describes
	// the source of the change events, _not_ the direction they are heading.
	// For example, upstream{Inf,Queue} receive updates that will result in the
	// syncer taking actions against the downstream cluster.
	upstreamInf     cache.SharedIndexInformer
	downstreamInf   cache.SharedIndexInformer
	upstreamQueue   workqueue.RateLimitingInterface
	downstreamQueue workqueue.RateLimitingInterface
	infDone         chan struct{}

	conflictErrors int

	done chan struct{} // Terminates all background processes.
}

func getStorageVersionIndex(crd crdtypes.CustomResourceDefinition) (int, error) {
	for ix, v := range crd.Spec.Versions {
		if v.Storage {
			return ix, nil
		}
	}
	return 0, fmt.Errorf("Invalid Custom Resource %s: no version with stored=true set", crd.ObjectMeta.Name)
}

func newCRSyncer(
	ctx context.Context,
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
			slog.Error("Value must be boolean",
				slog.String("Filter", annotationFilterByRobotName),
				slog.String("Target", crd.ObjectMeta.Name),
				slog.String("Got", filterByRobotValue))
		} else {
			filterByRobot = v
		}
	}
	versionIx, err := getStorageVersionIndex(crd)
	if err != nil {
		return nil, errors.Wrap(err, "Bad crd passed to newCRSyncer")
	}

	gvr := schema.GroupVersionResource{
		Group:    crd.Spec.Group,
		Version:  crd.Spec.Versions[versionIx].Name,
		Resource: crd.Spec.Names.Plural,
	}
	ns := ""
	if crd.Spec.Scope == crdtypes.NamespaceScoped {
		// TODO(https://github.com/googlecloudrobotics/core/issues/19): allow syncing CRs in other namespaces
		ns = "default"
	}
	s := &crSyncer{
		ctx:        ctx,
		crd:        crd,
		subtree:    annotations[annotationStatusSubtree],
		versionIx:  versionIx,
		upstream:   remote.Resource(gvr).Namespace(ns),
		downstream: local.Resource(gvr).Namespace(ns),
		done:       make(chan struct{}),
	}
	switch src := annotations[annotationSpecSource]; src {
	case "robot":
		s.clusterName = cloudClusterName
		// Swap upstream and downstream if the robot is the spec source.
		s.upstream, s.downstream = s.downstream, s.upstream
		// Use DefaultControllerRateLimiter for queue with destination robot and ItemFastSlowRateLimiter for queue with destination cloud to improve resilience regarding network errors
		// Upstream destination is robot cluster, downstream destination is cloud cluster
		s.upstreamQueue = workqueue.NewNamedRateLimitingQueue(workqueue.NewItemFastSlowRateLimiter(time.Millisecond*500, time.Second*5, 5), "upstream")
		s.downstreamQueue = workqueue.NewNamedRateLimitingQueue(workqueue.DefaultControllerRateLimiter(), "downstream")
	case "cloud":
		s.clusterName = fmt.Sprintf("robot-%s", robotName)
		// Use DefaultControllerRateLimiter for queue with destination robot and ItemFastSlowRateLimiter for queue with destination cloud to improve resilience regarding network errors
		// Upstream destination is robot cluster, downstream destination is cloud cluster
		s.upstreamQueue = workqueue.NewNamedRateLimitingQueue(workqueue.DefaultControllerRateLimiter(), "upstream")
		s.downstreamQueue = workqueue.NewNamedRateLimitingQueue(workqueue.NewItemFastSlowRateLimiter(time.Millisecond*500, time.Second*5, 5), "downstream")
	default:
		return nil, fmt.Errorf("unknown spec source %q", src)
	}
	if filterByRobot {
		if robotName != "" {
			s.labelSelector = labelRobotName + "=" + robotName
		} else {
			// TODO(fabxc): should this return an error instead?
			slog.Warn("request to filter by robot-name, but no robot-name was given to cr-syncer", slog.String("Requester", crd.ObjectMeta.Name))
		}
	}

	s.upstreamInf = s.newInformer(s.upstream)
	s.downstreamInf = s.newInformer(s.downstream)

	return s, nil
}

func (s *crSyncer) newInformer(client dynamic.ResourceInterface) cache.SharedIndexInformer {
	return cache.NewSharedIndexInformer(
		&cache.ListWatch{
			ListFunc: func(options metav1.ListOptions) (runtime.Object, error) {
				options.LabelSelector = s.labelSelector
				options.TimeoutSeconds = timeout
				return client.List(s.ctx, options)
			},
			WatchFunc: func(options metav1.ListOptions) (watch.Interface, error) {
				options.LabelSelector = s.labelSelector
				options.TimeoutSeconds = timeout
				return client.Watch(s.ctx, options)
			},
		},
		&unstructured.Unstructured{},
		resyncPeriod,
		nil,
	)
}

func (s *crSyncer) startInformers() error {
	if s.infDone != nil {
		return fmt.Errorf("informer for %s already started", s.crd.GetName())
	}
	s.infDone = make(chan struct{})

	go s.upstreamInf.Run(s.infDone)
	go s.downstreamInf.Run(s.infDone)

	if ok := cache.WaitForCacheSync(s.infDone, s.upstreamInf.HasSynced); !ok {
		return fmt.Errorf("stopped while syncing upstream informer for %s", s.crd.GetName())
	}
	if ok := cache.WaitForCacheSync(s.infDone, s.downstreamInf.HasSynced); !ok {
		return fmt.Errorf("stopped while syncing downstream informer for %s", s.crd.GetName())
	}
	s.setupInformerHandlers(s.upstreamInf, s.upstreamQueue, "upstream")
	s.setupInformerHandlers(s.downstreamInf, s.downstreamQueue, "downstream")

	return nil
}

func (s *crSyncer) stopInformers() {
	if s.infDone != nil {
		close(s.infDone)
		s.infDone = nil
	}
}

func (s *crSyncer) restartInformers() error {
	s.stopInformers()
	s.upstreamInf = s.newInformer(s.upstream)
	s.downstreamInf = s.newInformer(s.downstream)
	return s.startInformers()
}

func (s *crSyncer) setupInformerHandlers(
	inf cache.SharedIndexInformer,
	queue workqueue.RateLimitingInterface,
	direction string,
) {
	receive := func(obj interface{}, action string) {
		u := obj.(*unstructured.Unstructured)
		slog.Debug("Got Event",
			slog.String("Event", action),
			slog.String("Direction", direction),
			slog.String("Kind", u.GetKind()),
			slog.String("Name", u.GetName()),
			slog.String("Version", u.GetResourceVersion()))
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
	ctx context.Context,
	q workqueue.RateLimitingInterface,
	syncf func(string) error,
	qName string,
) bool {
	key, quit := q.Get()
	if quit {
		return false
	}
	defer q.Done(key)

	// Restart informers on too many conflict errors
	// client-go does not reliably recognize when watch calls are closed by remote API server
	// cr-syncer is able to detect that when updating CRs on remote API server when there are multiple subsequent conflict errors (HTTP 409)
	// like "...please apply your changes to the latest version and try again"
	// This could occur at watchers of single CRDs while others keep working. Thus, it is less resource intensive just restarting informers of the affected CRDs rather than whoel cr-syncer
	// Errors are counted in syncUpstream and syncDownstream functions
	if s.conflictErrors >= *conflictErrorLimit {
		slog.Info("Restarting informers because of too many conflict errors", slog.String("CRD", s.crd.GetName()))
		err := s.restartInformers()
		if err != nil {
			slog.Warn("Restarting informers failed", slog.String("CRD", s.crd.GetName()))
			q.AddRateLimited(key)
			return true
		} else {
			s.conflictErrors = 0
		}
	}

	ctx, err := tag.New(ctx, tag.Insert(tagEventSource, qName))
	if err != nil {
		panic(err)
	}
	err = syncf(key.(string))
	stats.Record(ctx, mSyncs.M(1))
	if err == nil {
		q.Forget(key)
		return true
	}
	// Synchronization failed, retry later.
	stats.Record(ctx, mSyncErrors.M(1))
	slog.Warn("Syncing key from queue failed",
		slog.Any("Key", key),
		slog.String("Queue", qName),
		ilog.Err(err))
	q.AddRateLimited(key)

	return true
}

func (s *crSyncer) run() {
	defer s.upstreamQueue.ShutDown()
	defer s.downstreamQueue.ShutDown()

	slog.Info("Starting syncer", slog.String("CRD", s.crd.GetName()))

	// Start informers that will populate their associated workqueue.
	if err := s.startInformers(); err != nil {
		slog.Warn("Starting informers failed", slog.String("CRD", s.crd.GetName()), ilog.Err(err))
		return
	}

	ctx, err := tag.New(context.Background(), tag.Insert(tagResource, s.crd.Name))
	if err != nil {
		panic(err)
	}
	// Process the upstream and downstream work queues.
	go func() {
		for s.processNextWorkItem(ctx, s.upstreamQueue, s.syncUpstream, "upstream") {
		}
	}()
	go func() {
		for s.processNextWorkItem(ctx, s.downstreamQueue, s.syncDownstream, "downstream") {
		}
	}()
	<-s.done
	// Close informers
	if s.infDone != nil {
		close(s.infDone)
	}
}

func (s *crSyncer) stop() {
	slog.Info("Stopping syncer", slog.String("CRD", s.crd.GetName()))
	close(s.done)
}

// syncDownstream reconciles state after receiving change events from the
// downstream cluster. It synchronizes the status from the downstream to the
// upstream cluster, and deletes orphaned downstream resources.
func (s *crSyncer) syncDownstream(key string) error {
	v := s.crd.Spec.Versions[s.versionIx]
	statusIsSubresource := v.Subresources != nil && v.Subresources.Status != nil
	// Get the downstream status (src) and upstream spec (dst).
	srcObj, srcExists, err := s.downstreamInf.GetIndexer().GetByKey(key)
	if err != nil {
		return fmt.Errorf("failed to retrieve resource for key %s: %s", key, err)
	}
	if !srcExists {
		// The downstream resource has been deleted: possibly because
		// the upstream resource was deleted and recreated. Add this to
		// the upstream queue so that syncUpstream() can check if it needs
		// to recreate the downstream resource.
		s.upstreamQueue.Add(key)
		return nil
	}
	src := srcObj.(*unstructured.Unstructured).DeepCopy()
	removeFinalizer(s.ctx, s.downstream, src, s.clusterName)

	dstObj, dstExists, err := s.upstreamInf.GetIndexer().GetByKey(key)
	if err != nil {
		return fmt.Errorf("failed to retrieve resource for key %s: %s", key, err)
	}
	// If the upstream resource no longer exists, delete the downstream
	// resource. Normally, this occurs when syncUpstream() handles the
	// upstream deletion, but if the resource was deleted when the robot
	// was offline, upstream doesn't know about the old resource and we'll
	// hit this condition.
	if !dstExists {
		if src.GetDeletionTimestamp() != nil {
			return nil // Already being deleted.
		}
		if err := s.downstream.Delete(s.ctx, src.GetName(), metav1.DeleteOptions{}); err != nil {
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
		copyStatus(dst, src)
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
		updated, err := s.upstream.UpdateStatus(s.ctx, dst, metav1.UpdateOptions{})
		if err != nil {
			// Count subsequent conflict errors
			if k8serrors.IsConflict(err) && s.clusterName != cloudClusterName {
				s.conflictErrors += 1
			}
			return newAPIErrorf(dst, "update status failed: %s", err)
		}
		dst = updated
	} else {
		updated, err := s.upstream.Update(s.ctx, dst, metav1.UpdateOptions{})
		if err != nil {
			// Count subsequent conflict errors
			if k8serrors.IsConflict(err) && s.clusterName != cloudClusterName {
				s.conflictErrors += 1
			}
			return newAPIErrorf(dst, "update failed: %s", err)
		}
		dst = updated
	}
	// Reset error count
	if s.clusterName != cloudClusterName {
		s.conflictErrors = 0
	}
	slog.Debug("Copied status to upstream",
		slog.String("Kind", src.GetKind()),
		slog.String("Name", src.GetName()),
		slog.Any("Source version", src.GetResourceVersion()),
		slog.Any("Destination version", dst.GetResourceVersion()))

	return nil
}

// syncUpstream reconciles the state after receiving a change event from upstream.
// It synchronizes the spec changes from upstream to the downstream cluster and propagates
// deletions.
func (s *crSyncer) syncUpstream(key string) error {
	// Get the upstream spec (src) and downstream status (dst).
	src := &unstructured.Unstructured{make(map[string]interface{})}
	dst := &unstructured.Unstructured{make(map[string]interface{})}
	srcObj, srcExists, err := s.upstreamInf.GetIndexer().GetByKey(key)
	if err != nil {
		return fmt.Errorf("failed to retrieve resource for key %s: %s", key, err)
	}
	if srcExists {
		src = srcObj.(*unstructured.Unstructured).DeepCopy()
		removeFinalizer(s.ctx, s.upstream, src, s.clusterName)
	}
	dstObj, dstExists, err := s.downstreamInf.GetIndexer().GetByKey(key)
	if err != nil {
		return fmt.Errorf("failed to retrieve resource for key %s: %s", key, err)
	}
	if dstExists {
		dst = dstObj.(*unstructured.Unstructured).DeepCopy()
	}

	// Check if the downstream resource (dst) should be created, updated,
	// or deleted. If we don't need to create/update dst, return early.
	var createOrUpdate func(*unstructured.Unstructured) (*unstructured.Unstructured, error)
	switch {
	case !srcExists && !dstExists:
		// Both deleted, nothing to do.
		return nil
	case srcExists && !dstExists:
		// Create object and set base fields.
		createOrUpdate = func(o *unstructured.Unstructured) (*unstructured.Unstructured, error) {
			o.SetGroupVersionKind(src.GroupVersionKind())
			o.SetNamespace(src.GetNamespace())
			o.SetName(src.GetName())
			// Copy upstream status on initial creation.
			o.Object["status"] = src.Object["status"]

			return s.downstream.Create(s.ctx, o, metav1.CreateOptions{})
		}
	case srcExists && dstExists:
		// Update dst.
		createOrUpdate = func(o *unstructured.Unstructured) (*unstructured.Unstructured, error) {
			return s.downstream.Update(s.ctx, o, metav1.UpdateOptions{})
		}
	case !srcExists && dstExists:
		// Delete dst.
		if err := s.downstream.Delete(s.ctx, dst.GetName(), metav1.DeleteOptions{}); err != nil {
			if isNotFoundError(err) {
				return nil
			}
			return newAPIErrorf(dst, "downstream delete failed: %s", err)
		}
		return nil
	default:
		slog.Error("unhandled condition",
			slog.Bool("srcExists", srcExists),
			slog.Bool("dstExists", dstExists))
		os.Exit(1)
		return nil
	}

	// Before creating/updating, check if deletion is in progress. This
	// is checked separately to src/dstExists for readability (hopefully).
	if src.GetDeletionTimestamp() != nil {
		if err := s.downstream.Delete(s.ctx, src.GetName(), metav1.DeleteOptions{}); err != nil {
			if isNotFoundError(err) {
				return nil
			}
			return newAPIErrorf(dst, "downstream delete failed: %s", err)
		}
		return nil
	}

	// Create/update dst with the labels+annotations+spec of src.
	dst.SetLabels(src.GetLabels())
	dst.SetAnnotations(src.GetAnnotations())
	dst.Object["spec"] = src.Object["spec"]

	// The remote-resource-version annotation is removed from dst to
	// prevent an infinite loop, because changing the annotation would
	// change the resource version.
	deleteAnnotation(dst, annotationResourceVersion)

	if _, err = createOrUpdate(dst); err != nil {
		// Count subsequent conflict errors
		if k8serrors.IsConflict(err) && s.clusterName == cloudClusterName {
			s.conflictErrors += 1
		}
		return newAPIErrorf(dst, "failed to create or update downstream: %s", err)
	}
	// Reset error count
	if s.clusterName == cloudClusterName {
		s.conflictErrors = 0
	}
	return nil
}

func isNotFoundError(err error) bool {
	status, ok := err.(*k8serrors.StatusError)
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
		slog.Warn("deriving key failed", ilog.Err(err))
		return k, false
	}
	return k, true
}

func setAnnotation(o *unstructured.Unstructured, key, value string) {
	annotations := o.GetAnnotations()
	if annotations == nil {
		annotations = make(map[string]string)
	}
	annotations[key] = value
	o.SetAnnotations(annotations)
}

func deleteAnnotation(o *unstructured.Unstructured, key string) {
	annotations := o.GetAnnotations()
	if annotations != nil {
		delete(annotations, key)
	}
	if len(annotations) > 0 {
		o.SetAnnotations(annotations)
	} else {
		o.SetAnnotations(nil)
	}

}

func copyStatus(dst, src *unstructured.Unstructured) {
	dst.Object["status"] = src.DeepCopy().Object["status"]
	// If this CR uses the observedGeneration convention, ensure that we
	// preserve the **equality** between generation and observedGeneration,
	// since the generations themselves will differ between local and remote.
	srcStatus, ok := src.Object["status"].(map[string]interface{})
	if !ok {
		// Status is not a dict => no observedGeneration.
		return
	}
	dstStatus := dst.Object["status"].(map[string]interface{})
	if srcOG, ok := srcStatus["observedGeneration"].(int64); ok {
		if src.GetGeneration() == srcOG {
			dstStatus["observedGeneration"] = dst.GetGeneration()
		} else {
			// The controller of this CR has not observed the latest generation.
			dstStatus["observedGeneration"] = 0
		}
	}
}
