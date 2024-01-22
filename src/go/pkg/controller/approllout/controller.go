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
	"log/slog"
	"net/http"
	"reflect"
	"sort"
	"strings"
	"time"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	registry "github.com/googlecloudrobotics/core/src/go/pkg/apis/registry/v1alpha1"
	"github.com/googlecloudrobotics/ilog"
	"github.com/pkg/errors"
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
	"sigs.k8s.io/controller-runtime/pkg/webhook/admission"
	"sigs.k8s.io/yaml"
)

const (
	fieldIndexOwners  = "metadata.ownerReferences.uid"
	fieldIndexAppName = "spec.appName"
	labelRobotName    = "cloudrobotics.com/robot-name"
)

// Add adds a controller for the AppRollout resource type
// to the manager and server.
func Add(ctx context.Context, mgr manager.Manager, baseValues chartutil.Values) error {
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

	err = mgr.GetCache().IndexField(ctx, &apps.ChartAssignment{}, fieldIndexOwners, indexOwnerReferences)
	if err != nil {
		return errors.Wrap(err, "add field indexer")
	}
	err = mgr.GetCache().IndexField(ctx, &apps.AppRollout{}, fieldIndexAppName, indexAppName)
	if err != nil {
		return errors.Wrap(err, "add field indexer")
	}

	err = c.Watch(
		source.Kind(mgr.GetCache(), &apps.AppRollout{}),
		&handler.EnqueueRequestForObject{},
	)
	if err != nil {
		return errors.Wrap(err, "watch AppRollouts")
	}
	// We don't trigger on ChartAssignment creations since it was either ourselves
	// or a CA we don't care about anyway.
	err = c.Watch(
		source.Kind(mgr.GetCache(), &apps.ChartAssignment{}),
		// We manually enqueue for the owner reference since handler.EnqueueRequestForOwner
		// does not work.
		// TODO: There is an associated bug in the controller-runtime but upgrading to include
		// https://github.com/kubernetes-sigs/controller-runtime/pull/274 did not resolve the issue.
		&handler.Funcs{
			DeleteFunc: func(ctx context.Context, evt event.DeleteEvent, q workqueue.RateLimitingInterface) {
				r.enqueueForOwner(evt.Object, q)
			},
			UpdateFunc: func(ctx context.Context, evt event.UpdateEvent, q workqueue.RateLimitingInterface) {
				r.enqueueForOwner(evt.ObjectNew, q)
			},
		},
	)
	if err != nil {
		return errors.Wrap(err, "watch ChartAssignments")
	}
	// Determining which rollouts are affected by a robot change is tedious.
	// We just enqueue all AppRollouts again.
	err = c.Watch(
		source.Kind(mgr.GetCache(), &registry.Robot{}),
		// We log robot events for now while b/125308238 persists.
		// To mitigate the effects we defer enqueueing in the delete handler
		// so the robot ideally reappeared before we reconcile.
		&handler.Funcs{
			CreateFunc: func(ctx context.Context, e event.CreateEvent, q workqueue.RateLimitingInterface) {
				slog.Info("AppRollout controller received create event", slog.String("Robot", e.Object.GetName()))
				r.enqueueAll(ctx, q)
			},
			UpdateFunc: func(ctx context.Context, e event.UpdateEvent, q workqueue.RateLimitingInterface) {
				// Robots don't have the status subresource enabled. Filter updates that didn't
				// change robot name or labels.
				change := !reflect.DeepEqual(e.ObjectOld.GetLabels(), e.ObjectNew.GetLabels())
				change = change || e.ObjectOld.GetName() != e.ObjectNew.GetName()
				if change {
					slog.Info("AppRollout controller received update event", slog.String("Robot", e.ObjectNew.GetName()))
					r.enqueueAll(ctx, q)
				}
			},
			DeleteFunc: func(ctx context.Context, e event.DeleteEvent, q workqueue.RateLimitingInterface) {
				slog.Info("AppRollout controller received delete event", slog.String("Robot", e.Object.GetName()))
				time.AfterFunc(3*time.Second, func() {
					r.enqueueAll(ctx, q)
				})
			},
		},
	)
	if err != nil {
		return errors.Wrap(err, "watch Robots")
	}
	err = c.Watch(
		source.Kind(mgr.GetCache(), &apps.App{}),
		&handler.Funcs{
			CreateFunc: func(ctx context.Context, e event.CreateEvent, q workqueue.RateLimitingInterface) {
				slog.Info("AppRollout controller received create event", slog.String("App", e.Object.GetName()))
				r.enqueueForApp(ctx, e.Object, q)
			},
			UpdateFunc: func(ctx context.Context, e event.UpdateEvent, q workqueue.RateLimitingInterface) {
				slog.Info("AppRollout controller received update event", slog.String("App", e.ObjectNew.GetName()))
				r.enqueueForApp(ctx, e.ObjectNew, q)
			},
			DeleteFunc: func(ctx context.Context, e event.DeleteEvent, q workqueue.RateLimitingInterface) {
				slog.Info("AppRollout controller received delete event", slog.String("App", e.Object.GetName()))
				r.enqueueForApp(ctx, e.Object, q)
			},
		},
	)
	if err != nil {
		return errors.Wrap(err, "watch Apps")
	}
	return nil
}

// enqueueForApp enqueues all AppRollouts for the given app.
func (r *Reconciler) enqueueForApp(ctx context.Context, m metav1.Object, q workqueue.RateLimitingInterface) {
	var rollouts apps.AppRolloutList
	err := r.kube.List(ctx, &rollouts, kclient.MatchingFields(map[string]string{fieldIndexAppName: m.GetName()}))
	if err != nil {
		slog.Error("List AppRollouts failed", slog.String("appName", m.GetName()), ilog.Err(err))
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
				NamespacedName: types.NamespacedName{Name: or.Name},
			})
		}
	}
}

// enqueueAll enqueues all AppRollouts.
func (r *Reconciler) enqueueAll(ctx context.Context, q workqueue.RateLimitingInterface) {
	var rollouts apps.AppRolloutList
	err := r.kube.List(ctx, &rollouts)
	if err != nil {
		slog.Error("List AppRollouts failed", ilog.Err(err))
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

func (r *Reconciler) Reconcile(ctx context.Context, req reconcile.Request) (reconcile.Result, error) {
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
	slog.Info("Reconcile AppRollout",
		slog.String("Name", ar.Name),
		slog.String("Version", ar.ResourceVersion))

	// Apply spec.
	var (
		curCAs apps.ChartAssignmentList
		al     apps.AppList
		robots registry.RobotList
	)

	ar.Status.ObservedGeneration = ar.Generation
	ar.Status.Assignments = 0
	ar.Status.SettledAssignments = 0
	ar.Status.ReadyAssignments = 0
	ar.Status.FailedAssignments = 0

	// TODO(coconutruben): consider moving these into a testable function.
	// Moving them into generateChartAssignments requires rewriting the
	// existing tests.
	err := r.kube.List(ctx, &curCAs, kclient.MatchingFields(map[string]string{fieldIndexOwners: string(ar.UID)}))
	if err != nil {
		return reconcile.Result{}, errors.Wrapf(err, "list ChartAssignments for owner UID %s", ar.UID)
	}

	if err := r.kube.List(ctx, &robots); err != nil {
		return reconcile.Result{}, errors.Wrap(err, "list all Robots")
	}

	if err := r.kube.List(ctx, &al, kclient.MatchingLabels{labelAppName: ar.Spec.AppName}); err != nil {
		return reconcile.Result{}, errors.Wrap(err, "list all App Versions")
	}

	// There might be old Apps laying around that do not conform to this
	// methodology yet. However, those Apps also do not use the version
	// mechanism. Therefore, we can just look for that App once, and put
	// it into our map if it's not there yet.
	appFound := func(name string) bool {
		for idx := range al.Items {
			if al.Items[idx].Name == name {
				return true
			}
		}
		return false
	}
	if !appFound(ar.Spec.AppName) {
		// We might not need the canonical app at all, if all the rollout
		// entries are asking for versioned apps. Do not fail here yet,
		// if we needed it, it will fail in generateChartAssignments
		app := apps.App{}
		if err := r.kube.Get(ctx, kclient.ObjectKey{Name: ar.Spec.AppName}, &app); err == nil {
			// If we're here, the App is both:
			// - app.Name == ar.Spec.AppName.
			// - not found using the app-name label. This object is a copy so
			// we can just write the label in there, so generateChartAssignments
			// can safely assume the label is always there.
			if app.Labels == nil {
				app.Labels = map[string]string{}
			}
			app.Labels[labelAppName] = app.Name
			app.Labels[labelAppVersion] = ""
			al.Items = append(al.Items, app)
		}
	}

	wantCAs, err := generateChartAssignments(al.Items, robots.Items, ar, r.baseValues)
	if err != nil {
		if _, ok := errors.Cause(err).(errRobotSelectorOverlap); ok {
			return reconcile.Result{}, r.updateErrorStatus(ctx, ar, err.Error())
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
			slog.Info("Created ChartAssignment", slog.String("Name", ca.Name))
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
		slog.Info("Updated ChartAssignment", slog.String("Name", ca.Name))
	}
	// Delete obsolete assignments.
	for _, ca := range dropCAs {
		if err := r.kube.Delete(ctx, &ca); err != nil {
			return reconcile.Result{}, errors.Wrapf(err, "delete ChartAssignment %q", ca.Name)
		}
		slog.Info("Deleted ChartAssignment", slog.String("Name", ca.Name))
	}

	setStatus(ar, len(wantCAs), curCAs.Items)

	if err := r.kube.Status().Update(ctx, ar); err != nil {
		return reconcile.Result{}, errors.Wrap(err, "update status")
	}
	return reconcile.Result{}, nil
}

func (r *Reconciler) updateErrorStatus(ctx context.Context, ar *apps.AppRollout, msg string) error {
	setCondition(ar, apps.AppRolloutConditionSettled, core.ConditionFalse, msg)
	if err := r.kube.Status().Update(ctx, ar); err != nil {
		return errors.Wrap(err, "update status")
	}
	return nil
}

func setStatus(ar *apps.AppRollout, numWantCAs int, curCAs []apps.ChartAssignment) {
	// Update status.
	ar.Status.Assignments = int64(numWantCAs)

	for _, ca := range curCAs {
		switch ca.Status.Phase {
		case apps.ChartAssignmentPhaseReady:
			ar.Status.ReadyAssignments++
			ar.Status.SettledAssignments++
		case apps.ChartAssignmentPhaseSettled:
			ar.Status.SettledAssignments++
		case apps.ChartAssignmentPhaseFailed:
			ar.Status.FailedAssignments++
		}
	}
	if got, want := ar.Status.SettledAssignments, ar.Status.Assignments; got == want {
		setCondition(ar, apps.AppRolloutConditionSettled, core.ConditionTrue, "")
	} else {
		setCondition(ar, apps.AppRolloutConditionSettled, core.ConditionFalse,
			fmt.Sprintf("%d/%d ChartAssignments settled", got, want))
	}
	if got, want := ar.Status.ReadyAssignments, ar.Status.Assignments; got == want {
		setCondition(ar, apps.AppRolloutConditionReady, core.ConditionTrue, "")
	} else {
		setCondition(ar, apps.AppRolloutConditionReady, core.ConditionFalse,
			fmt.Sprintf("%d/%d ChartAssignments ready", got, want))
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
		if c.Status != s || c.Message != msg {
			c.LastUpdateTime = now
		}
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
	al []apps.App,
	robots []registry.Robot,
	rollout *apps.AppRollout,
	baseValues chartutil.Values,
) ([]*apps.ChartAssignment, error) {

	var (
		// Different entries might request different app versions. This map
		// is used to only retrieve them once.
		appVersions = map[string]*apps.App{}
		cas         []*apps.ChartAssignment
		// Robots that matched selectors for the rollout and which will be
		// passed to the cloud chart.
		selectedRobots = map[string]*registry.Robot{}
	)

	for _, app := range al {
		v, ok := app.Labels[labelAppVersion]
		if !ok {
			// If only app-name is defined, it is an unversioned app.
			v = ""
		}
		if _, ok := appVersions[v]; ok {
			slog.Info("App already known. Going to ignore App Object for the same app/version",
				slog.String("Known Name", rollout.Spec.AppName),
				slog.String("Known Version", v),
				slog.String("App Object", app.Name))
		} else {
			// only add to map if this is the first time we're adding this
			// version for the app. The validator should ensure that this
			// overwrite cannot happen, but apps might be legacy, or have
			// bypassed validation.
			appVersions[v] = &app
		}
	}
	for _, rcomp := range rollout.Spec.Robots {
		robots, err := matchingRobots(robots, rcomp.Selector)
		if err != nil {
			return nil, errors.Wrap(err, "select robots")
		}
		// map is populated by for all the rcomp.Version, no need to check ok
		app, ok := appVersions[rcomp.Version]
		if !ok {
			return nil, fmt.Errorf("no App %q (Version: %q) found", rollout.Spec.AppName, rcomp.Version)
		}
		comps := app.Spec.Components
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
	// The cloud has has no version, just the canonical version. We might
	// not have it due to no robot using it.
	if app, ok := appVersions[""]; ok {
		comps := app.Spec.Components
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
	} else if rollout.Spec.Cloud.Values != nil {
		slog.Info("No canonical version of App. There won't be a Cloud ChartAssignment. AppRollout defines cloud values.",
			slog.String("App Name", rollout.Spec.AppName),
			slog.String("Rollout Name", rollout.Name))
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
	for k, v := range rollout.Annotations {
		if k != core.LastAppliedConfigAnnotation {
			setAnnotation(&ca.ObjectMeta, k, v)
		}
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

// matchingRobots returns the subset of robots that pass the given robot selector.
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

func setAnnotation(o *metav1.ObjectMeta, k, v string) {
	if o.Annotations == nil {
		o.Annotations = map[string]string{}
	}
	o.Annotations[k] = v
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
func indexOwnerReferences(o kclient.Object) (vs []string) {
	ca := o.(*apps.ChartAssignment)
	for _, or := range ca.OwnerReferences {
		vs = append(vs, string(or.UID))
	}
	return vs
}

func indexAppName(o kclient.Object) []string {
	ar := o.(*apps.AppRollout)
	return []string{ar.Spec.AppName}
}

const (
	// canonical name of an app
	labelAppName = "cloudrobotics.com/app-name"
	// version of that app. Note, the default version of the app has
	// a version label of ""
	labelAppVersion = "cloudrobotics.com/app-version"
)

// NewAppValidationWebhook returns a new webhook that validates Apps.
//
// This pertains to multiple versions of the same app, so that the labels
// defined above are in sync with the name of the App.
// The policy is
// - an unversioned app defines
//   - cloudrobotics.com/app-name
//   - (optionally): cloudrobotics.com/app-version with a "" value
//     this must match the name of the object
//
// - a versioned app defines
//   - cloudrobotics.com/app-name
//   - cloudrobotics.com/app-version
//     the name of the App object must match LOWERCASE([app-name].v[app-version])
func NewAppValidationWebhook(mgr manager.Manager) *admission.Webhook {
	return &admission.Webhook{Handler: newAppValidator(mgr.GetScheme())}
}

// appValidator implements a validation webhook.
type appValidator struct {
	decoder runtime.Decoder
}

func newAppValidator(sc *runtime.Scheme) *appValidator {
	return &appValidator{
		decoder: serializer.NewCodecFactory(sc).UniversalDeserializer(),
	}
}

func (v *appValidator) Handle(_ context.Context, req admission.Request) admission.Response {
	cur := &apps.App{}
	if err := runtime.DecodeInto(v.decoder, req.AdmissionRequest.Object.Raw, cur); err != nil {
		return admission.Errored(http.StatusBadRequest, err)
	}
	if err := appValidate(cur); err != nil {
		return admission.Denied(err.Error())
	}
	return admission.Allowed("")
}

func appValidate(cur *apps.App) error {
	name := cur.Name
	appName, anok := cur.Labels[labelAppName]
	appVersion, avok := cur.Labels[labelAppVersion]
	if anok {
		if avok && appVersion != "" {
			// both name and version are defined
			ename := strings.ToLower(fmt.Sprintf("%s.v%s", appName, appVersion))
			if ename != name {
				return fmt.Errorf("%q=%q, %q=%q: expected object name %q, got %q", labelAppName, appName, labelAppVersion, appVersion, ename, name)
			}
		} else {
			// only name is defined
			if appName != name {
				return fmt.Errorf("%q=%q, undefined %q: expected object name %q, got %q", labelAppName, appName, labelAppVersion, appName, name)
			}
		}
	}
	// neither is defined, we're dealing with a legacy app
	return nil
}

// NewAppRolloutValidationWebhook returns a new webhook that validates AppRollouts.
func NewAppRolloutValidationWebhook(mgr manager.Manager) *admission.Webhook {
	return &admission.Webhook{Handler: newAppRolloutValidator(mgr.GetScheme())}
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

func (v *appRolloutValidator) Handle(_ context.Context, req admission.Request) admission.Response {
	cur := &apps.AppRollout{}

	if err := runtime.DecodeInto(v.decoder, req.AdmissionRequest.Object.Raw, cur); err != nil {
		return admission.Errored(http.StatusBadRequest, err)
	}
	if err := appRolloutValidate(cur); err != nil {
		return admission.Denied(err.Error())
	}
	return admission.Allowed("")
}

func appRolloutValidate(cur *apps.AppRollout) error {
	if cur.Spec.AppName == "" {
		return errors.New("app name missing")
	}
	errs := validation.NameIsDNSSubdomain(cur.Spec.AppName, false)
	if len(errs) > 0 {
		return errors.Errorf("validate app name: %s", strings.Join(errs, ", "))
	}
	if _, ok := cur.Spec.Cloud.Values["robots"]; ok {
		return errors.Errorf(".spec.cloud.values.robots is a reserved field and must not be set")
	}
	for i, r := range cur.Spec.Robots {
		if _, ok := r.Values["robot"]; ok {
			return errors.Errorf(".spec.robots[].values.robot is a reserved field and must not be set")
		}
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
