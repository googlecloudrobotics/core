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

package approllout

import (
	"context"
	"fmt"
	"log"
	"net/http"
	"sort"
	"strings"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	registry "github.com/googlecloudrobotics/core/src/go/pkg/apis/registry/v1alpha1"
	"github.com/pkg/errors"
	admissionregistration "k8s.io/api/admissionregistration/v1beta1"
	k8serrors "k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/api/validation"
	meta "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/labels"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/runtime/serializer"
	"k8s.io/helm/pkg/chartutil"
	kclient "sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/controller"
	"sigs.k8s.io/controller-runtime/pkg/handler"
	"sigs.k8s.io/controller-runtime/pkg/manager"
	"sigs.k8s.io/controller-runtime/pkg/reconcile"
	"sigs.k8s.io/controller-runtime/pkg/source"
	"sigs.k8s.io/controller-runtime/pkg/webhook"
	"sigs.k8s.io/controller-runtime/pkg/webhook/admission"
	"sigs.k8s.io/controller-runtime/pkg/webhook/admission/builder"
	admissiontypes "sigs.k8s.io/controller-runtime/pkg/webhook/admission/types"
)

// Add adds a controller for the AppRollout resource type
// to the manager and server.
func Add(mgr manager.Manager) error {
	c, err := controller.New("approllout", mgr, controller.Options{
		Reconciler: &Reconciler{
			kube: mgr.GetClient(),
		},
	})
	if err != nil {
		return errors.Wrap(err, "create controller")
	}
	err = c.Watch(
		&source.Kind{Type: &apps.AppRollout{}},
		&handler.EnqueueRequestForObject{},
	)
	if err != nil {
		return errors.Wrap(err, "watch AppRollouts")
	}
	return nil
}

// Reconciler provides an idempotent function that brings the cluster into a
// state consistent with the specification of an AppRollout.
type Reconciler struct {
	kube kclient.Client
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
	log.Printf("Reconcile AppRollout %q", ar.Name)
	return reconcile.Result{}, nil
}

type errRobotSelectorOverlap string

func (r errRobotSelectorOverlap) Error() string {
	return fmt.Sprintf("robot %q was selected multiple times", r)
}

// generateChartAssignments returns a list of all cloud and robot ChartAssignments
// for the given app, its rollout, and set of robots.
func generateChartAssignments(
	app *apps.App,
	rollout *apps.AppRollout,
	allRobots []*registry.Robot,
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
		for _, r := range robots {
			// No robot must be selected multiple times.
			if _, ok := selectedRobots[r.Name]; ok {
				return nil, errRobotSelectorOverlap(r.Name)
			}
			selectedRobots[r.Name] = r

			cas = append(cas, newRobotChartAssignment(r, app, rollout, &rcomp, baseValues))
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
	ca.Spec.ClusterName = robot.Name
	if spec.Version != "" {
		ca.Spec.Chart.Version = spec.Version
	}

	vals := chartutil.Values{}
	vals.MergeInto(values)
	vals.MergeInto(chartutil.Values(spec.Values))

	ca.Spec.Chart.Values = apps.ConfigValues(vals)

	return ca
}

// newChartAssignments returns a new ChartAssignments that's initialized with
// all values that are fixed for the app, its rollout, and component.
func newBaseChartAssignment(app *apps.App, rollout *apps.AppRollout, comp *apps.AppComponent) *apps.ChartAssignment {
	var ca apps.ChartAssignment

	ca.Labels = rollout.Labels
	ca.Annotations = rollout.Annotations
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
func matchingRobots(robots []*registry.Robot, sel *apps.RobotSelector) ([]*registry.Robot, error) {
	if sel.Any {
		return robots, nil
	}
	selector, err := meta.LabelSelectorAsSelector(&sel.LabelSelector)
	if err != nil {
		return nil, err
	}
	var res []*registry.Robot
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
		return fmt.Sprintf("%s-%s.%s", rollout, typ, robot)
	}
	return fmt.Sprintf("%s-%s", rollout, typ)
}

// robotValues is the struct that is passed into the cloud chart configuration
// for each robot matched by a rollout.
type robotValues struct {
	Name string `json:"name"`
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
	}
	return nil
}
