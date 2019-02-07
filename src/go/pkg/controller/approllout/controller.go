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
	"log"
	"net/http"
	"strings"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"github.com/pkg/errors"
	admissionregistration "k8s.io/api/admissionregistration/v1beta1"
	k8serrors "k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/api/validation"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/runtime/serializer"
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
