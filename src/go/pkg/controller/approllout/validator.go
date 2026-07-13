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

package approllout

import (
	"context"
	"fmt"
	"net/http"
	"strings"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"k8s.io/apimachinery/pkg/api/validation"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/runtime/serializer"
	"sigs.k8s.io/controller-runtime/pkg/manager"
	"sigs.k8s.io/controller-runtime/pkg/webhook/admission"
)

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
		return fmt.Errorf("spec.appName is missing for AppRollout %q", cur.Name)
	}
	errs := validation.NameIsDNSSubdomain(cur.Spec.AppName, false)
	if len(errs) > 0 {
		return fmt.Errorf("validate app name: %s", strings.Join(errs, ", "))
	}
	if _, ok := cur.Spec.Cloud.Values["robots"]; ok {
		return fmt.Errorf(".spec.cloud.values.robots is a reserved field and must not be set")
	}
	for i, r := range cur.Spec.Robots {
		if _, ok := r.Values["robot"]; ok {
			return fmt.Errorf(".spec.robots[].values.robot is a reserved field and must not be set")
		}
		if r.Selector == nil {
			return fmt.Errorf("no selector provided for robots %d", i)
		}
		// Reject if a selector has neither a matcher nor `any` set.
		// This mostly helps catching missing `matchLabels`.
		if r.Selector.Any == nil && r.Selector.LabelSelector == nil {
			return fmt.Errorf("empty selector for robots %d (matchLabels not specified?)", i)
		}
	}
	return nil
}
