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

package v1alpha1

import (
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/runtime"
)

// +genclient
// +genclient:nonNamespaced
// +k8s:deepcopy-gen:interfaces=k8s.io/apimachinery/pkg/runtime.Object

type App struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`

	Spec AppSpec `json:"spec,omitempty"`
}

// +genclient:nonNamespaced
// +k8s:deepcopy-gen:interfaces=k8s.io/apimachinery/pkg/runtime.Object

type AppList struct {
	metav1.TypeMeta
	metav1.ListMeta
	Items []App
}

type AppSpec struct {
	Repository string        `json:"repository"`
	Version    string        `json:"version"`
	Components AppComponents `json:"components"`
}

type AppComponents struct {
	Cloud AppComponent `json:"cloud,omitempty"`
	Robot AppComponent `json:"robot,omitempty"`
}

type AppComponent struct {
	Name   string `json:"name,omitempty"`
	Inline string `json:"inline,omitempty"`
}

// +genclient
// +genclient:nonNamespaced
// +k8s:deepcopy-gen:interfaces=k8s.io/apimachinery/pkg/runtime.Object

type AppRollout struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`

	Spec   AppRolloutSpec   `json:"spec,omitempty"`
	Status AppRolloutStatus `json:"status,omitempty"`
}

// +genclient:nonNamespaced
// +k8s:deepcopy-gen:interfaces=k8s.io/apimachinery/pkg/runtime.Object

type AppRolloutList struct {
	metav1.TypeMeta
	metav1.ListMeta
	Items []AppRollout
}

type AppRolloutSpec struct {
	AppName string                `json:"appName,omitempty"`
	Cloud   AppRolloutSpecCloud   `json:"cloud,omitempty"`
	Robots  []AppRolloutSpecRobot `json:"robots,omitempty"`
}

type AppRolloutSpecCloud struct {
	Values ConfigValues `json:"values,omitempty"`
}

type AppRolloutSpecRobot struct {
	Selector *RobotSelector `json:"selector,omitempty"`

	Values  ConfigValues `json:"values,omitempty"`
	Version string       `json:"version,omitempty"`
}

type RobotSelector struct {
	*metav1.LabelSelector

	Any *bool `json:"any,omitempty"`
}

type AppRolloutStatus struct {
	ObservedGeneration int64                 `json:"observedGeneration,omitempty"`
	Conditions         []AppRolloutCondition `json:"conditions,omitempty"`
	Assignments        int64                 `json:"assignments"`
	SettledAssignments int64                 `json:"settledAssignments"`
	FailedAssignments  int64                 `json:"failedAssignments"`
}

type AppRolloutCondition struct {
	Type               AppRolloutConditionType `json:"type"`
	Status             corev1.ConditionStatus  `json:"status"`
	LastUpdateTime     metav1.Time             `json:"lastUpdateTime,omitempty"`
	LastTransitionTime metav1.Time             `json:"lastTransitionTime,omitempty"`
	Message            string                  `json:"message,omitempty"`
}

type AppRolloutConditionType string

const (
	AppRolloutConditionSettled AppRolloutConditionType = "Settled"
)

// +genclient
// +genclient:nonNamespaced
// +k8s:deepcopy-gen:interfaces=k8s.io/apimachinery/pkg/runtime.Object

type ChartAssignment struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`

	Spec   ChartAssignmentSpec   `json:"spec,omitempty"`
	Status ChartAssignmentStatus `json:"status,omitempty"`
}

// +genclient:nonNamespaced
// +k8s:deepcopy-gen:interfaces=k8s.io/apimachinery/pkg/runtime.Object

type ChartAssignmentList struct {
	metav1.TypeMeta
	metav1.ListMeta
	Items []ChartAssignment
}

type ChartAssignmentSpec struct {
	ClusterName   string        `json:"clusterName"`
	NamespaceName string        `json:"namespaceName"`
	Chart         AssignedChart `json:"chart"`
}

type AssignedChart struct {
	Repository string       `json:"repository,omitempty"`
	Name       string       `json:"name,omitempty"`
	Version    string       `json:"version,omitempty"`
	Inline     string       `json:"inline,omitempty"`
	Values     ConfigValues `json:"values,omitempty"`
}

type ConfigValues map[string]interface{}

// DeepCopy is an explicit override since the deepcopy generator cannot
// deal with empty interfaces.
func (in ConfigValues) DeepCopy() ConfigValues {
	return runtime.DeepCopyJSON(in)
}

type ChartAssignmentStatus struct {
	ObservedGeneration int64                      `json:"observedGeneration,omitempty"`
	Phase              ChartAssignmentPhase       `json:"phase,omitempty"`
	Conditions         []ChartAssignmentCondition `json:"conditions,omitempty"`
	Helm               ChartAssignmentStatusHelm  `json:"helm,omitempty"`
}

type ChartAssignmentStatusHelm struct {
	Revision    int32  `json:"revision,omitempty"`
	Description string `json:"description,omitempty"`
}

type ChartAssignmentPhase string

const (
	// Accepted is set once the controller has observed the CA and started
	// taking action.
	ChartAssignmentPhaseAccepted     ChartAssignmentPhase = "Accepted"
	ChartAssignmentPhaseLoadingChart                      = "LoadingChart"
	ChartAssignmentPhaseInstalling                        = "Installing"
	ChartAssignmentPhaseUpdating                          = "Updating"
	ChartAssignmentPhaseDeleting                          = "Deleting"
	ChartAssignmentPhaseSettled                           = "Settled"
	ChartAssignmentPhaseDeleted                           = "Deleted"
	ChartAssignmentPhaseFailed                            = "Failed"
)

type ChartAssignmentCondition struct {
	Type               ChartAssignmentConditionType `json:"type"`
	Status             corev1.ConditionStatus       `json:"status"`
	LastUpdateTime     metav1.Time                  `json:"lastUpdateTime,omitempty"`
	LastTransitionTime metav1.Time                  `json:"lastTransitionTime,omitempty"`
	Message            string                       `json:"message,omitempty"`
}

type ChartAssignmentConditionType string

const ChartAssignmentConditionSettled ChartAssignmentConditionType = "Settled"
