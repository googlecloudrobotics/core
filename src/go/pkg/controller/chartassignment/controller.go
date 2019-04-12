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

package chartassignment

import (
	"context"
	"fmt"
	"log"
	"net/http"
	"strings"
	"time"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"github.com/googlecloudrobotics/core/src/go/pkg/gcr"
	"github.com/pkg/errors"
	admissionregistration "k8s.io/api/admissionregistration/v1beta1"
	core "k8s.io/api/core/v1"
	k8serrors "k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/api/validation"
	meta "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/runtime/serializer"
	record "k8s.io/client-go/tools/record"
	"k8s.io/client-go/util/workqueue"
	hclient "k8s.io/helm/pkg/helm"
	kclient "sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/controller"
	"sigs.k8s.io/controller-runtime/pkg/event"
	"sigs.k8s.io/controller-runtime/pkg/handler"
	"sigs.k8s.io/controller-runtime/pkg/manager"
	"sigs.k8s.io/controller-runtime/pkg/reconcile"
	"sigs.k8s.io/controller-runtime/pkg/source"
	"sigs.k8s.io/controller-runtime/pkg/webhook"
	"sigs.k8s.io/controller-runtime/pkg/webhook/admission"
	"sigs.k8s.io/controller-runtime/pkg/webhook/admission/builder"
	admissiontypes "sigs.k8s.io/controller-runtime/pkg/webhook/admission/types"
)

const (
	// In-cluster hostname of tiller in a standard installation.
	DefaultTillerHost = "tiller-deploy.kube-system.svc:44134"

	fieldIndexNamespace = "spec.namespaceName"
)

// Add adds a controller and validation webhook for the ChartAssignment resource type
// to the manager and server.
// Handled ChartAssignments are filtered by the provided cluster.
func Add(mgr manager.Manager, cluster, tillerHost string) error {
	if tillerHost == "" {
		tillerHost = DefaultTillerHost
	}
	r := &Reconciler{
		kube:     mgr.GetClient(),
		helm:     hclient.NewClient(hclient.Host(tillerHost)),
		recorder: mgr.GetRecorder("chartassignment-controller"),
		cluster:  cluster,
	}
	var err error
	r.releases, err = newReleases(mgr.GetConfig(), r.helm, r.recorder)
	if err != nil {
		return err
	}

	c, err := controller.New("chartassignment", mgr, controller.Options{
		Reconciler: r,
	})
	if err != nil {
		return err
	}
	err = mgr.GetCache().IndexField(&apps.ChartAssignment{}, fieldIndexNamespace,
		func(o runtime.Object) []string {
			return []string{o.(*apps.ChartAssignment).Spec.NamespaceName}
		},
	)
	if err != nil {
		return errors.Wrap(err, "add field indexer")
	}
	err = c.Watch(
		&source.Kind{Type: &apps.ChartAssignment{}},
		&handler.EnqueueRequestForObject{},
	)
	if err != nil {
		return err
	}
	err = c.Watch(
		&source.Kind{Type: &core.Pod{}},
		&handler.Funcs{
			CreateFunc: func(e event.CreateEvent, q workqueue.RateLimitingInterface) {
				r.enqueueForPod(e.Meta, q)
			},
			UpdateFunc: func(e event.UpdateEvent, q workqueue.RateLimitingInterface) {
				r.enqueueForPod(e.MetaNew, q)
			},
			DeleteFunc: func(e event.DeleteEvent, q workqueue.RateLimitingInterface) {
				r.enqueueForPod(e.Meta, q)
			},
		},
	)
	if err != nil {
		return errors.Wrap(err, "watch Apps")
	}
	return nil
}

func (r *Reconciler) enqueueForPod(m meta.Object, q workqueue.RateLimitingInterface) {
	var cas apps.ChartAssignmentList
	err := r.kube.List(context.TODO(), kclient.MatchingField(fieldIndexNamespace, m.GetNamespace()), &cas)
	if err != nil {
		log.Printf("List ChartAssignments for namespace %s failed: %s", m.GetNamespace(), err)
		return
	}
	for _, ca := range cas.Items {
		q.Add(reconcile.Request{
			NamespacedName: kclient.ObjectKey{Name: ca.Name},
		})
	}
}

// Reconciler provides an idempotent function that brings the cluster into a
// state consistent with the specification of a ChartAssignment.
type Reconciler struct {
	kube     kclient.Client
	helm     hclient.Interface
	recorder record.EventRecorder
	cluster  string // Cluster for which to handle ChartAssignments.
	releases *releases
}

// Reconcile creates and updates a Helm release for the given chart assignment.
// It rolls back releases to the previous revision if an upgrade failed.
// It continously requeues the ChartAssignment for reconciliation to monitor the
// status of the Helm release.
func (r *Reconciler) Reconcile(req reconcile.Request) (reconcile.Result, error) {
	ctx := context.TODO()

	var as apps.ChartAssignment
	err := r.kube.Get(ctx, req.NamespacedName, &as)

	if as.Spec.ClusterName != r.cluster {
		return reconcile.Result{}, nil
	}
	if k8serrors.IsNotFound(err) {
		// Assignment was already deleted. We did all required cleanup
		// when removing the finalizer. Thus, there's nothing to do.
		log.Printf("ChartAssignment %q no longer exists, skipping reconciliation...", req.NamespacedName)
		return reconcile.Result{}, nil
	} else if err != nil {
		return reconcile.Result{}, fmt.Errorf("getting ChartAssignment %q failed: %s", req, err)
	}
	return r.reconcile(ctx, &as)
}

const (
	// The finalizer that's applied to assignments to block their garbage collection
	// until the Helm release is deleted.
	finalizer = "helm.apps.cloudrobotics.com"
	// Requeue interval when the underlying Helm release is not in a stable state yet.
	requeueFast = 3 * time.Second
	// Requeue interval after the underlying Helm release reached a stable state.
	requeueSlow = 3 * time.Minute
	// Timeout seconds after which Tiller should abort any attempted release operations.
	tillerTimeout = 600
)

func (r *Reconciler) ensureNamespace(ctx context.Context, as *apps.ChartAssignment) (*core.Namespace, error) {
	// Create application namespace if it doesn't exist.
	var ns core.Namespace
	err := r.kube.Get(ctx, kclient.ObjectKey{Name: as.Spec.NamespaceName}, &ns)

	if err != nil && !k8serrors.IsNotFound(err) {
		return nil, fmt.Errorf("getting Namespace %q failed: %s", as.Spec.NamespaceName, err)
	}
	if ns.DeletionTimestamp != nil {
		return nil, fmt.Errorf("namespace %q is marked for deletion, skipping", as.Spec.NamespaceName)
	}

	createNamespace := k8serrors.IsNotFound(err)
	ns.Name = as.Spec.NamespaceName

	// Add ourselves to the owners if we aren't already.
	_true := true
	added := setOwnerReference(&ns.ObjectMeta, meta.OwnerReference{
		APIVersion:         as.APIVersion,
		Kind:               as.Kind,
		Name:               as.Name,
		UID:                as.UID,
		BlockOwnerDeletion: &_true,
	})
	if !added {
		return &ns, nil
	}
	if createNamespace {
		return &ns, r.kube.Create(ctx, &ns)
	}
	return &ns, r.kube.Update(ctx, &ns)
}

// ensureServiceAccount makes sure we have an image pull secret for gcr.io inside the apps namespace
// and the default service account configured to use it. This is needed to make apps work that
// reference images from a private container registry.
// TODO(ensonic): Put this behind a flag to only do this as needed.
func (r *Reconciler) ensureServiceAccount(ctx context.Context, ns *core.Namespace, as *apps.ChartAssignment) error {
	if r.cluster == "cloud" {
		// We don't need any of this for cloud charts.
		return nil
	}

	// Copy imagePullSecret from 'default' namespace, since service accounts cannot reference
	// secrets in other namespaces.
	var secret core.Secret
	err := r.kube.Get(ctx, kclient.ObjectKey{Namespace: as.Spec.NamespaceName, Name: gcr.SecretName}, &secret)
	if k8serrors.IsNotFound(err) {
		err = r.kube.Get(ctx, kclient.ObjectKey{Namespace: "default", Name: gcr.SecretName}, &secret)
		if k8serrors.IsNotFound(err) {
			log.Printf("Failed to get Secret \"default:%s\" (this is expected when simulating a robot on GKE)", gcr.SecretName)
			return nil
		} else if err != nil {
			return fmt.Errorf("getting Secret \"default:%s\" failed: %s", gcr.SecretName, err)
		}
		// Don't reuse full metadata in created secret.
		secret.ObjectMeta = meta.ObjectMeta{
			Namespace: ns.Name,
			Name:      gcr.SecretName,
		}
		err = r.kube.Create(ctx, &secret)
		if err != nil {
			return fmt.Errorf("creating Secret \"%s:%s\" failed: %s", as.Spec.NamespaceName, gcr.SecretName, err)
		}
	}

	// Configure the default service account in the namespace.
	var sa core.ServiceAccount
	err = r.kube.Get(ctx, kclient.ObjectKey{Namespace: as.Spec.NamespaceName, Name: "default"}, &sa)
	if err != nil {
		return fmt.Errorf("getting ServiceAccount \"%s:default\" failed: %s", as.Spec.NamespaceName, err)
	}

	// Only add the secret once.
	ips := core.LocalObjectReference{Name: gcr.SecretName}
	found := false
	for _, s := range sa.ImagePullSecrets {
		if s == ips {
			found = true
			break
		}
	}
	if !found {
		sa.ImagePullSecrets = append(sa.ImagePullSecrets, ips)
	}
	return r.kube.Update(ctx, &sa)
}

func (r *Reconciler) reconcile(ctx context.Context, as *apps.ChartAssignment) (reconcile.Result, error) {
	// If we are scheduled for deletion, delete the Helm release and drop our
	// finalizer so garbage collection can continue.
	if as.DeletionTimestamp != nil {
		log.Printf("Ensure ChartAssignment %q cleanup", as.Name)

		if err := r.ensureDeleted(ctx, as); err != nil {
			return reconcile.Result{}, fmt.Errorf("ensure deleted: %s", err)
		}
		if err := r.setStatus(ctx, as); err != nil {
			return reconcile.Result{}, fmt.Errorf("set status: %s", err)
		}
		// Requeue to track deletion progress.
		return reconcile.Result{Requeue: true, RequeueAfter: requeueFast}, nil
	}

	ns, err := r.ensureNamespace(ctx, as)
	if err != nil {
		return reconcile.Result{}, fmt.Errorf("ensure namespace: %s", err)
	}
	if err := r.ensureServiceAccount(ctx, ns, as); err != nil {
		return reconcile.Result{}, fmt.Errorf("ensure service-account: %s", err)
	}
	// Ensure a finalizer on the ChartAssignment so we don't get deleted before
	// we've properly deleted the associated Helm release.
	if !stringsContain(as.Finalizers, finalizer) {
		as.Finalizers = append(as.Finalizers, finalizer)
		if err := r.kube.Update(ctx, as); err != nil {
			return reconcile.Result{}, err
		}
	}

	r.releases.ensureUpdated(as)

	if err := r.setStatus(ctx, as); err != nil {
		return reconcile.Result{}, errors.Wrap(err, "update status")
	}
	// Quickly requeue for status updates when deployment is in progress.
	switch as.Status.Phase {
	case apps.ChartAssignmentPhaseSettled, apps.ChartAssignmentPhaseFailed:
		return reconcile.Result{Requeue: true, RequeueAfter: requeueSlow}, nil
	}
	return reconcile.Result{Requeue: true, RequeueAfter: requeueFast}, nil

}

func condition(b bool) core.ConditionStatus {
	if b {
		return core.ConditionTrue
	}
	return core.ConditionFalse
}

func (r *Reconciler) setStatus(ctx context.Context, as *apps.ChartAssignment) error {
	// Remove old status conditions.
	// TODO: Should be removed eventually so LastTransitionTime of conditions is accurate.
	as.Status.Conditions = nil

	status, ok := r.releases.status(as.Name)
	if !ok {
		return nil
	}

	as.Status.ObservedGeneration = as.Generation
	as.Status.Phase = status.phase
	as.Status.Helm.Revision = status.revision
	as.Status.Helm.Description = status.description

	if c := condition(status.phase == apps.ChartAssignmentPhaseSettled); status.err == nil {
		setCondition(as, apps.ChartAssignmentConditionSettled, c, "")
	} else {
		setCondition(as, apps.ChartAssignmentConditionSettled, c, status.err.Error())
	}

	// Determine readiness based on pods in the app namespace being ready.
	// This is an incomplete heuristic but it should catch the vast majority of errors.
	var pods core.PodList
	if err := r.kube.List(ctx, kclient.InNamespace(as.Spec.NamespaceName), &pods); err != nil {
		return errors.Wrap(err, "list pods")
	}
	ready, total := 0, len(pods.Items)

	for _, p := range pods.Items {
		switch p.Status.Phase {
		case core.PodRunning, core.PodSucceeded:
			ready++
		}
	}
	// Readiness is only given if the release is settled to begin with.
	if status.phase != apps.ChartAssignmentPhaseSettled {
		setCondition(as, apps.ChartAssignmentConditionReady, core.ConditionFalse,
			"Helm release not settled yet")
	} else {
		if ready == total {
			as.Status.Phase = apps.ChartAssignmentPhaseReady
		}
		setCondition(as, apps.ChartAssignmentConditionReady, condition(ready == total),
			fmt.Sprintf("%d/%d pods are running or succeeded", ready, total))
	}
	return r.kube.Status().Update(ctx, as)
}

// ensureDeleted ensures that the Helm release is deleted and the finalizer gets removed.
func (r *Reconciler) ensureDeleted(ctx context.Context, as *apps.ChartAssignment) error {
	r.releases.ensureDeleted(as)
	status, ok := r.releases.status(as.Name)
	if !ok {
		return fmt.Errorf("release status not found")
	}

	if status.phase != apps.ChartAssignmentPhaseDeleted {
		// Deletion still in progress, check again later.
		return nil
	}
	if !stringsContain(as.Finalizers, finalizer) {
		return nil
	}
	as.Finalizers = stringsDelete(as.Finalizers, finalizer)
	if err := r.kube.Update(ctx, as); err != nil {
		return fmt.Errorf("update failed: %s", err)
	}
	return nil
}

func stringsContain(list []string, s string) bool {
	for _, x := range list {
		if x == s {
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

// setOwnerReference ensures the owner reference is set and returns true if it did
// not exist before. Existing references are detected based on the UID field.
func setOwnerReference(om *meta.ObjectMeta, ref meta.OwnerReference) bool {
	for i, or := range om.OwnerReferences {
		if ref.UID == or.UID {
			om.OwnerReferences[i] = ref
			return false
		}
	}
	om.OwnerReferences = append(om.OwnerReferences, ref)
	return true
}

// inCondition returns true if the ChartAssignment has a condition of the given
// type in state true.
func inCondition(as *apps.ChartAssignment, c apps.ChartAssignmentConditionType) bool {
	for _, cond := range as.Status.Conditions {
		if cond.Type == c && cond.Status == core.ConditionTrue {
			return true
		}
	}
	return false
}

// setCondition adds or updates a condition. Existing conditions are detected
// based on the Type field.
func setCondition(as *apps.ChartAssignment, t apps.ChartAssignmentConditionType, v core.ConditionStatus, msg string) {
	now := meta.Now()

	for i, c := range as.Status.Conditions {
		if c.Type != t {
			continue
		}
		// Update existing condition.
		c.LastUpdateTime = now
		if c.Status != v {
			c.LastTransitionTime = now
		}
		c.Message = msg
		c.Status = v
		as.Status.Conditions[i] = c
		return
	}
	// Condition set for the first time.
	as.Status.Conditions = append(as.Status.Conditions, apps.ChartAssignmentCondition{
		Type:               t,
		LastUpdateTime:     now,
		LastTransitionTime: now,
		Status:             v,
		Message:            msg,
	})
}

// NewValidationWebhook returns a new webhook that validates ChartAssignments.
func NewValidationWebhook(mgr manager.Manager) (webhook.Webhook, error) {
	return builder.NewWebhookBuilder().
		Name("chartassignments.apps.cloudrobotics.com").
		Validating().
		Operations(admissionregistration.Create, admissionregistration.Update).
		FailurePolicy(admissionregistration.Fail).
		WithManager(mgr).
		ForType(&apps.ChartAssignment{}).
		Handlers(newChartAssignmentValidator(mgr.GetScheme())).
		Build()
}

// chartAssignmentValidator implements a validation webhook.
type chartAssignmentValidator struct {
	decoder runtime.Decoder
}

func newChartAssignmentValidator(sc *runtime.Scheme) *chartAssignmentValidator {
	return &chartAssignmentValidator{
		decoder: serializer.NewCodecFactory(sc).UniversalDeserializer(),
	}
}

func (v *chartAssignmentValidator) Handle(_ context.Context, req admissiontypes.Request) admissiontypes.Response {
	cur := &apps.ChartAssignment{}
	old := &apps.ChartAssignment{}

	if err := runtime.DecodeInto(v.decoder, req.AdmissionRequest.Object.Raw, cur); err != nil {
		return admission.ErrorResponse(http.StatusBadRequest, err)
	}
	if len(req.AdmissionRequest.OldObject.Raw) > 0 {
		if err := runtime.DecodeInto(v.decoder, req.AdmissionRequest.OldObject.Raw, old); err != nil {
			return admission.ErrorResponse(http.StatusBadRequest, err)
		}
	} else {
		old = nil
	}
	if err := validate(cur, old); err != nil {
		return admission.ValidationResponse(false, err.Error())
	}
	return admission.ValidationResponse(true, "")
}

func validate(cur, old *apps.ChartAssignment) error {
	if cur.Spec.ClusterName == "" {
		return fmt.Errorf("cluster name missing")
	}
	if cur.Spec.NamespaceName == "" {
		return fmt.Errorf("namespace name missing")
	}
	errs := validation.ValidateNamespaceName(cur.Spec.NamespaceName, false)
	if len(errs) > 0 {
		return fmt.Errorf("invalid namespace name: %s", strings.Join(errs, ", "))
	}
	errs = validation.ValidateClusterName(cur.Spec.ClusterName, false)
	if len(errs) > 0 {
		return fmt.Errorf("invalid cluster name: %s", strings.Join(errs, ", "))
	}
	if old != nil {
		if cur.Spec.NamespaceName != old.Spec.NamespaceName {
			return fmt.Errorf("target namespace name must not be changed")
		}
		if cur.Spec.ClusterName != old.Spec.ClusterName {
			return fmt.Errorf("target cluster name must not be changed")
		}
	}
	c := cur.Spec.Chart
	if c.Inline != "" {
		if c.Repository != "" || c.Name != "" || c.Version != "" {
			return fmt.Errorf("chart repository, name, and version must be empty for inline charts")
		}
	} else if c.Repository == "" || c.Name == "" || c.Version == "" {
		return fmt.Errorf("non-inline chart must be fully specified")
	}
	return nil
}
