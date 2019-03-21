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

package approllout

import (
	"bytes"
	"context"
	"fmt"
	"log"
	"net/http"
	"reflect"
	"sort"
	"strings"
	"time"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	registry "github.com/googlecloudrobotics/core/src/go/pkg/apis/registry/v1alpha1"
	"github.com/pkg/errors"
	admissionregistration "k8s.io/api/admissionregistration/v1beta1"
	core "k8s.io/api/core/v1"
	k8serrors "k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/api/validation"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/labels"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/runtime/serializer"
	"k8s.io/apimachinery/pkg/types"
	"k8s.io/client-go/util/workqueue"
	"k8s.io/helm/pkg/chartutil"
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
	"sigs.k8s.io/yaml"
)

const (
	fieldIndexOwners  = "metadata.ownerReferences.uid"
	fieldIndexAppName = "spec.appName"
	labelRobotName    = "cloudrobotics.com/robot-name"
)

// Add adds a controller for the AppRollout resource type
// to the manager and server.
func Add(mgr manager.Manager, baseValues chartutil.Values) error {
	r := &Reconciler{
		kube:       mgr.GetClient(),
		baseValues: baseValues,
	}
	c, err := controller.New("approllout", mgr, controller.Options{
		Reconciler: r,
	})
	if err != nil {
		return errors.Wrap(err, "create controller")
	}

	err = mgr.GetCache().IndexField(&apps.ChartAssignment{}, fieldIndexOwners, indexOwnerReferences)
	if err != nil {
		return errors.Wrap(err, "add field indexer")
	}
	err = mgr.GetCache().IndexField(&apps.AppRollout{}, fieldIndexAppName, indexAppName)
	if err != nil {
		return errors.Wrap(err, "add field indexer")
	}

	err = c.Watch(
		&source.Kind{Type: &apps.AppRollout{}},
		&handler.EnqueueRequestForObject{},
	)
	if err != nil {
		return errors.Wrap(err, "watch AppRollouts")
	}
	// We don't trigger on ChartAssignment creations since it was either ourselves
	// or a CA we don't care about anyway.
	err = c.Watch(
		&source.Kind{Type: &apps.ChartAssignment{}},
		// We manually enqueue for the owner reference since handler.EnqueueRequestForOwner
		// does not work.
		// TODO: There is an associated bug in the controller-runtime but upgrading to include
		// https://github.com/kubernetes-sigs/controller-runtime/pull/274 did not resolve the issue.
		&handler.Funcs{
			DeleteFunc: func(evt event.DeleteEvent, q workqueue.RateLimitingInterface) {
				r.enqueueForOwner(evt.Meta, q)
			},
			UpdateFunc: func(evt event.UpdateEvent, q workqueue.RateLimitingInterface) {
				r.enqueueForOwner(evt.MetaNew, q)
			},
		},
	)
	if err != nil {
		return errors.Wrap(err, "watch ChartAssignments")
	}
	// Determining which rollouts are affected by a robot change is tedious.
	// We just enqueue all AppRollouts again.
	err = c.Watch(
		&source.Kind{Type: &registry.Robot{}},
		// We log robot events for now while b/125308238 persists.
		// To mitigate the effects we defer enqueueing in the delete handler
		// so the robot ideally reappeared before we reconcile.
		&handler.Funcs{
			CreateFunc: func(e event.CreateEvent, q workqueue.RateLimitingInterface) {
				log.Printf("AppRollout controller received create event for Robot %q", e.Meta.GetName())
				r.enqueueAll(q)
			},
			UpdateFunc: func(e event.UpdateEvent, q workqueue.RateLimitingInterface) {
				// Robots don't have the status subresource enabled. Filter updates that didn't
				// change robot name or labels.
				change := !reflect.DeepEqual(e.MetaOld.GetLabels(), e.MetaNew.GetLabels())
				change = change || e.MetaOld.GetName() != e.MetaNew.GetName()
				if change {
					r.enqueueAll(q)
				}
			},
			DeleteFunc: func(e event.DeleteEvent, q workqueue.RateLimitingInterface) {
				log.Printf("AppRollout controller received delete event for Robot %q", e.Meta.GetName())
				time.AfterFunc(3*time.Second, func() {
					r.enqueueAll(q)
				})
			},
		},
	)
	if err != nil {
		return errors.Wrap(err, "watch Robots")
	}
	err = c.Watch(
		&source.Kind{Type: &apps.App{}},
		&handler.Funcs{
			CreateFunc: func(e event.CreateEvent, q workqueue.RateLimitingInterface) {
				r.enqueueForApp(e.Meta, q)
			},
			UpdateFunc: func(e event.UpdateEvent, q workqueue.RateLimitingInterface) {
				r.enqueueForApp(e.MetaNew, q)
			},
			DeleteFunc: func(e event.DeleteEvent, q workqueue.RateLimitingInterface) {
				r.enqueueForApp(e.Meta, q)
			},
		},
	)
	if err != nil {
		return errors.Wrap(err, "watch Apps")
	}
	return nil
}

// enqueueForApp enqueues all AppRollouts for the given app.
func (r *Reconciler) enqueueForApp(m metav1.Object, q workqueue.RateLimitingInterface) {
	var rollouts apps.AppRolloutList
	err := r.kube.List(context.TODO(), kclient.MatchingField(fieldIndexAppName, m.GetName()), &rollouts)
	if err != nil {
		log.Printf("List AppRollouts for appName %s failed: %s", m.GetName(), err)
		return
	}
	for _, ar := range rollouts.Items {
		q.Add(reconcile.Request{
			NamespacedName: types.NamespacedName{Name: ar.Name},
		})
	}
}

// enqueueForOwner enqueues AppRollouts that are listed in the owner references
// of the given resource metadata.
func (r *Reconciler) enqueueForOwner(m metav1.Object, q workqueue.RateLimitingInterface) {
	for _, or := range m.GetOwnerReferences() {
		if or.APIVersion == "apps.cloudrobotics.com/v1alpha1" && or.Kind == "AppRollout" {
			q.Add(reconcile.Request{
				types.NamespacedName{Name: or.Name},
			})
		}
	}
}

// enqueueAll enqueues all AppRollouts.
func (r *Reconciler) enqueueAll(q workqueue.RateLimitingInterface) {
	var rollouts apps.AppRolloutList
	err := r.kube.List(context.TODO(), nil, &rollouts)
	if err != nil {
		log.Printf("List AppRollouts failed: %s", err)
		return
	}
	for _, ar := range rollouts.Items {
		q.Add(reconcile.Request{
			NamespacedName: types.NamespacedName{Name: ar.Name},
		})
	}
}

// Reconciler provides an idempotent function that brings the cluster into a
// state consistent with the specification of an AppRollout.
type Reconciler struct {
	kube       kclient.Client
	baseValues chartutil.Values
}

func (r *Reconciler) Reconcile(req reconcile.Request) (reconcile.Result, error) {
	ctx := context.TODO()

	var ar apps.AppRollout
	err := r.kube.Get(ctx, req.NamespacedName, &ar)

	if k8serrors.IsNotFound(err) {
		// AppRollout was already deleted, nothing to do.
		return reconcile.Result{}, nil
	} else if err != nil {
		return reconcile.Result{}, errors.Wrapf(err, "get AppRollout %q", req)
	}
	return r.reconcile(ctx, &ar)
}

func (r *Reconciler) reconcile(ctx context.Context, ar *apps.AppRollout) (reconcile.Result, error) {
	log.Printf("Reconcile AppRollout %q (version: %s)", ar.Name, ar.ResourceVersion)

	// Apply spec.
	var (
		app    apps.App
		curCAs apps.ChartAssignmentList
		robots registry.RobotList
	)
	ar.Status.ObservedGeneration = ar.Generation
	ar.Status.Assignments = 0
	ar.Status.UpdatedAssignments = 0

	err := r.kube.Get(ctx, kclient.ObjectKey{Name: ar.Spec.AppName}, &app)
	if err != nil {
		r.updateErrorStatus(ctx, ar, err.Error())
		return reconcile.Result{}, errors.Wrapf(err, "get app %q", ar.Spec.AppName)
	}
	err = r.kube.List(ctx, kclient.MatchingField(fieldIndexOwners, string(ar.UID)), &curCAs)
	if err != nil {
		return reconcile.Result{}, errors.Wrapf(err, "list ChartAssignments for owner UID %s", ar.UID)
	}
	// NOTE(freinartz): consider pushing this down to generateChartAssignments
	// and passing the robot selectors directly to the client.
	if err := r.kube.List(ctx, nil, &robots); err != nil {
		return reconcile.Result{}, errors.Wrap(err, "list all Robots")
	}

	wantCAs, err := generateChartAssignments(&app, ar, robots.Items, r.baseValues)
	if err != nil {
		if _, ok := errors.Cause(err).(errRobotSelectorOverlap); ok {
			r.updateErrorStatus(ctx, ar, err.Error())
		}
		return reconcile.Result{}, errors.Wrap(err, "generate ChartAssignments")
	}

	// ChartAssignments that are no longer wanted. We pre-populate it with
	// all existing CAs and remove those that we want to keep
	dropCAs := map[string]apps.ChartAssignment{}

	for _, ca := range curCAs.Items {
		dropCAs[ca.Name] = ca
	}
	// Create or update ChartAssignments. Only update ChartAssignments if the rollout's
	// spec or labels have been updated.
	for _, ca := range wantCAs {
		_true := true
		setOwnerReference(&ca.ObjectMeta, metav1.OwnerReference{
			APIVersion:         ar.APIVersion,
			Kind:               ar.Kind,
			Name:               ar.Name,
			UID:                ar.UID,
			BlockOwnerDeletion: &_true,
			Controller:         &_true,
		})
		prev, exists := dropCAs[ca.Name]
		delete(dropCAs, ca.Name)

		if !exists {
			if err := r.kube.Create(ctx, ca); err != nil {
				return reconcile.Result{}, errors.Wrapf(err, "create ChartAssignment %q", ca.Name)
			}
			log.Printf("Created ChartAssignment %q", ca.Name)
			continue
		}
		if changed, err := chartAssignmentChanged(&prev, ca); err != nil {
			return reconcile.Result{}, errors.Wrap(err, "check ChartAssignment changed")
		} else if !changed {
			continue
		}
		ca.ResourceVersion = prev.ResourceVersion
		if err := r.kube.Update(ctx, ca); err != nil {
			return reconcile.Result{}, errors.Wrapf(err, "update ChartAssignment %q", ca.Name)
		}
		log.Printf("Updated ChartAssignment %q", ca.Name)
	}
	// Delete obsolete assignments.
	for _, ca := range dropCAs {
		if err := r.kube.Delete(ctx, &ca); err != nil {
			return reconcile.Result{}, errors.Wrapf(err, "delete ChartAssignment %q", ca.Name)
		}
		log.Printf("Deleted ChartAssignment %q", ca.Name)
	}
	// Update status.
	ar.Status.Assignments = int64(len(wantCAs))
	for _, ca := range curCAs.Items {
		if ca.Status.DeployedRevision == ca.Status.DesiredRevision {
			ar.Status.UpdatedAssignments += 1
		}
	}
	setCondition(ar, apps.AppRolloutConditionSettled, core.ConditionTrue, "")

	if err := r.kube.Status().Update(ctx, ar); err != nil {
		return reconcile.Result{}, errors.Wrap(err, "update status")
	}
	return reconcile.Result{}, nil
}

func (r *Reconciler) updateErrorStatus(ctx context.Context, ar *apps.AppRollout, msg string) {
	setCondition(ar, apps.AppRolloutConditionSettled, core.ConditionFalse, msg)
	if err := r.kube.Status().Update(ctx, ar); err != nil {
		log.Printf("Failed to set AppRollout status: %q\n", err)
	}
}

// setCondition adds or updates a condition. Existing conditions are detected based on the Type field.
func setCondition(ar *apps.AppRollout, t apps.AppRolloutConditionType, s core.ConditionStatus, msg string) {
	now := metav1.Now()

	for i, c := range ar.Status.Conditions {
		if c.Type != t {
			continue
		}
		// Update existing condition.
		c.LastUpdateTime = now
		if c.Status != s {
			c.LastTransitionTime = now
		}
		c.Message = msg
		c.Status = s
		ar.Status.Conditions[i] = c
		return
	}
	// Condition set for the first time.
	ar.Status.Conditions = append(ar.Status.Conditions, apps.AppRolloutCondition{
		Type:               t,
		LastUpdateTime:     now,
		LastTransitionTime: now,
		Status:             s,
		Message:            msg,
	})
}

// chartAssignmentChanged returns true if the CA's labels, annotations, or spec changed.
func chartAssignmentChanged(prev, cur *apps.ChartAssignment) (bool, error) {
	if !reflect.DeepEqual(prev.Labels, cur.Labels) {
		return true, nil
	}
	if !reflect.DeepEqual(prev.Annotations, cur.Annotations) {
		return true, nil
	}
	prevSpec, err := yaml.Marshal(prev.Spec)
	if err != nil {
		return false, err
	}
	curSpec, err := yaml.Marshal(cur.Spec)
	if err != nil {
		return false, err
	}
	return !bytes.Equal(prevSpec, curSpec), nil
}

type errRobotSelectorOverlap string

func (r errRobotSelectorOverlap) Error() string {
	return fmt.Sprintf("robot %q was selected multiple times", string(r))
}

// generateChartAssignments returns a list of all cloud and robot ChartAssignments
// for the given app, its rollout, and set of robots.
func generateChartAssignments(
	app *apps.App,
	rollout *apps.AppRollout,
	allRobots []registry.Robot,
	baseValues chartutil.Values,
) ([]*apps.ChartAssignment, error) {
	var (
		cas   []*apps.ChartAssignment
		comps = app.Spec.Components
		// Robots that matched selectors for the rollout and which will be
		// passed to the cloud chart.
		selectedRobots = map[string]*registry.Robot{}
	)
	for _, rcomp := range rollout.Spec.Robots {
		robots, err := matchingRobots(allRobots, rcomp.Selector)
		if err != nil {
			return nil, errors.Wrap(err, "select robots")
		}
		for i := range robots {
			// Ensure we don't pass a pointer to the most recent loop item.
			r := &robots[i]
			// No robot must be selected multiple times.
			if _, ok := selectedRobots[r.Name]; ok {
				return nil, errRobotSelectorOverlap(r.Name)
			}
			selectedRobots[r.Name] = r

			if comps.Robot.Name != "" || comps.Robot.Inline != "" {
				cas = append(cas, newRobotChartAssignment(r, app, rollout, &rcomp, baseValues))
			}
		}
	}
	if comps.Cloud.Name != "" || comps.Cloud.Inline != "" {
		// Turn robot map into a sorted slice so we produce deterministic outputs.
		// (Go randomizes map iteration.)
		robots := make([]*registry.Robot, 0, len(selectedRobots))
		for _, r := range selectedRobots {
			robots = append(robots, r)
		}
		sort.Slice(robots, func(i, j int) bool {
			return robots[i].Name < robots[j].Name
		})
		cas = append(cas, newCloudChartAssignment(app, rollout, baseValues, robots...))
	}
	sort.Slice(cas, func(i, j int) bool {
		return cas[i].Name < cas[j].Name
	})
	return cas, nil
}

// newCloudChartAssignment generates a new ChartAssignment for the cloud cluster
// from an app, it's rollout, a set of base configuration values,
// and a list of robots matched by the rollout.
func newCloudChartAssignment(
	app *apps.App,
	rollout *apps.AppRollout,
	values chartutil.Values,
	robots ...*registry.Robot,
) *apps.ChartAssignment {
	ca := newBaseChartAssignment(app, rollout, &app.Spec.Components.Cloud)

	ca.Name = chartAssignmentName(rollout.Name, compTypeCloud, "")
	ca.Spec.ClusterName = "cloud"

	// Generate robot values list that's injected into the cloud chart.
	var robotValuesList []robotValues
	for _, r := range robots {
		robotValuesList = append(robotValuesList, robotValues{
			Name: r.Name,
		})
	}
	vals := chartutil.Values{}
	vals.MergeInto(values)
	vals.MergeInto(chartutil.Values(rollout.Spec.Cloud.Values))
	vals.MergeInto(chartutil.Values{"robots": robotValuesList})

	ca.Spec.Chart.Values = apps.ConfigValues(vals)

	return ca
}

// newRobotChartAssignment generates a new ChartAssignment for a robot cluster
// from an app, its rollout, and a set of base configuration values.
func newRobotChartAssignment(
	robot *registry.Robot,
	app *apps.App,
	rollout *apps.AppRollout,
	spec *apps.AppRolloutSpecRobot,
	values chartutil.Values,
) *apps.ChartAssignment {
	ca := newBaseChartAssignment(app, rollout, &app.Spec.Components.Robot)

	ca.Name = chartAssignmentName(rollout.Name, compTypeRobot, robot.Name)
	setLabel(&ca.ObjectMeta, labelRobotName, robot.Name)

	ca.Spec.ClusterName = robot.Name
	if spec.Version != "" {
		ca.Spec.Chart.Version = spec.Version
	}

	vals := chartutil.Values{}
	vals.MergeInto(values)
	vals.MergeInto(chartutil.Values(spec.Values))
	vals.MergeInto(chartutil.Values{"robot": robotValues{Name: robot.Name}})

	ca.Spec.Chart.Values = apps.ConfigValues(vals)

	return ca
}

// newChartAssignments returns a new ChartAssignments that's initialized with
// all values that are fixed for the app, its rollout, and component.
func newBaseChartAssignment(app *apps.App, rollout *apps.AppRollout, comp *apps.AppComponent) *apps.ChartAssignment {
	var ca apps.ChartAssignment

	// Clone labels and annotations. Just setting the map reference
	// would cause the map to be shared across objects.
	for k, v := range rollout.Labels {
		setLabel(&ca.ObjectMeta, k, v)
	}
	ca.Spec.NamespaceName = appNamespaceName(rollout.Name)

	if comp.Name != "" {
		ca.Spec.Chart = apps.AssignedChart{
			Repository: app.Spec.Repository,
			Version:    app.Spec.Version,
			Name:       comp.Name,
		}
	}
	ca.Spec.Chart.Inline = comp.Inline

	return &ca
}

// matchingRopbots returns the subset of robots that pass the given robot selector.
// It returns an error if the selector is invalid.
func matchingRobots(robots []registry.Robot, sel *apps.RobotSelector) ([]registry.Robot, error) {
	if sel.Any != nil && *sel.Any {
		return robots, nil
	}
	if sel.LabelSelector == nil {
		return nil, nil
	}
	selector, err := metav1.LabelSelectorAsSelector(sel.LabelSelector)
	if err != nil {
		return nil, err
	}
	var res []registry.Robot
	for _, r := range robots {
		if selector.Matches(labels.Set(r.Labels)) {
			res = append(res, r)
		}
	}
	return res, nil
}

func appNamespaceName(rollout string) string {
	return fmt.Sprintf("app-%s", rollout)
}

type componentType string

const (
	compTypeRobot componentType = "robot"
	compTypeCloud               = "cloud"
)

func chartAssignmentName(rollout string, typ componentType, robot string) string {
	if robot != "" {
		return fmt.Sprintf("%s-%s-%s", rollout, typ, robot)
	}
	return fmt.Sprintf("%s-%s", rollout, typ)
}

// robotValues is the struct that is passed into the cloud chart configuration
// for each robot matched by a rollout.
type robotValues struct {
	Name string `json:"name"`
}

func setLabel(o *metav1.ObjectMeta, k, v string) {
	if o.Labels == nil {
		o.Labels = map[string]string{}
	}
	o.Labels[k] = v
}

// setOwnerReference adds or updates an owner reference. Existing references
// are detected based on the UID field.
func setOwnerReference(om *metav1.ObjectMeta, ref metav1.OwnerReference) {
	for i, or := range om.OwnerReferences {
		if ref.UID == or.UID {
			om.OwnerReferences[i] = ref
			return
		}
	}
	om.OwnerReferences = append(om.OwnerReferences, ref)
}

// indexOwnerReferences indexes resources by the UIDs of their owner references.
func indexOwnerReferences(o runtime.Object) (vs []string) {
	ca := o.(*apps.ChartAssignment)
	for _, or := range ca.OwnerReferences {
		vs = append(vs, string(or.UID))
	}
	return vs
}

func indexAppName(o runtime.Object) []string {
	ar := o.(*apps.AppRollout)
	return []string{ar.Spec.AppName}
}

// NewValidationWebhook returns a new webhook that validates AppRollouts.
func NewValidationWebhook(mgr manager.Manager) (webhook.Webhook, error) {
	return builder.NewWebhookBuilder().
		Name("approllouts.apps.cloudrobotics.com").
		Validating().
		Operations(admissionregistration.Create, admissionregistration.Update).
		FailurePolicy(admissionregistration.Fail).
		WithManager(mgr).
		ForType(&apps.AppRollout{}).
		Handlers(newAppRolloutValidator(mgr.GetScheme())).
		Build()
}

// appRolloutValidator implements a validation webhook.
type appRolloutValidator struct {
	decoder runtime.Decoder
}

func newAppRolloutValidator(sc *runtime.Scheme) *appRolloutValidator {
	return &appRolloutValidator{
		decoder: serializer.NewCodecFactory(sc).UniversalDeserializer(),
	}
}

func (v *appRolloutValidator) Handle(_ context.Context, req admissiontypes.Request) admissiontypes.Response {
	cur := &apps.AppRollout{}

	if err := runtime.DecodeInto(v.decoder, req.AdmissionRequest.Object.Raw, cur); err != nil {
		return admission.ErrorResponse(http.StatusBadRequest, err)
	}
	if err := validate(cur); err != nil {
		return admission.ValidationResponse(false, err.Error())
	}
	return admission.ValidationResponse(true, "")
}

func validate(cur *apps.AppRollout) error {
	if cur.Spec.AppName == "" {
		return errors.New("app name missing")
	}
	errs := validation.NameIsDNSSubdomain(cur.Spec.AppName, false)
	if len(errs) > 0 {
		return errors.Errorf("validate app name: %s", strings.Join(errs, ", "))
	}
	for i, r := range cur.Spec.Robots {
		if r.Selector == nil {
			return errors.Errorf("no selector provided for robots %d", i)
		}
		// Reject if a selector has neither a matcher nor `any` set.
		// This mostly helps catching missing `matchLabels`.
		if r.Selector.Any == nil && r.Selector.LabelSelector == nil {
			return errors.Errorf("empty selector for robots %d (matchLabels not specified?)", i)
		}
	}
	return nil
}
