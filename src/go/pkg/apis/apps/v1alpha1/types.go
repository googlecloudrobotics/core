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

type ResourceSet struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`

	Spec   ResourceSetSpec   `json:"spec"`
	Status ResourceSetStatus `json:"status"`
}

// +k8s:deepcopy-gen:interfaces=k8s.io/apimachinery/pkg/runtime.Object

type ResourceSetList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`

	Items []ResourceSet `json:"items"`
}

type ResourceSetSpec struct {
	Resources []ResourceSetSpecGroup `json:"resources"`
}

type ResourceSetStatus struct {
	Phase      ResourceSetPhase         `json:"phase,omitempty"`
	StartedAt  metav1.Time              `json:"startedAt,omitempty"`
	FinishedAt metav1.Time              `json:"finishedAt,omitempty"`
	Applied    []ResourceSetStatusGroup `json:"applied,omitempty"`
	Failed     []ResourceSetStatusGroup `json:"failed,omitempty"`
}

type ResourceSetSpecGroup struct {
	Group   string        `json:"group,omitempty"` // Is empty for core APIs.
	Version string        `json:"version"`
	Kind    string        `json:"kind"`
	Items   []ResourceRef `json:"items"`
}

type ResourceSetStatusGroup struct {
	Group   string           `json:"group,omitempty"` // Is empty for core APIs.
	Version string           `json:"version"`
	Kind    string           `json:"kind"`
	Items   []ResourceStatus `json:"items"`
}

type ResourceRef struct {
	Namespace string `json:"namespace,omitempty"`
	Name      string `json:"name"`
}

type ResourceStatus struct {
	Namespace  string         `json:"namespace,omitempty"`
	Name       string         `json:"name"`
	Action     ResourceAction `json:"action"`
	UID        string         `json:"uid,omitempty"`
	Generation int64          `json:"generation,omitempty"`
	Error      string         `json:"error,omitempty"`
}

type ResourceSetPhase string

const (
	ResourceSetPhasePending ResourceSetPhase = "Pending"
	ResourceSetPhaseFailed  ResourceSetPhase = "Failed"
	ResourceSetPhaseSettled ResourceSetPhase = "Settled"
)

type ResourceAction string

const (
	ResourceActionNone    ResourceAction = "None"
	ResourceActionCreate  ResourceAction = "Create"
	ResourceActionUpdate  ResourceAction = "Update"
	ResourceActionReplace ResourceAction = "Replace"
)

// +genclient
// +genclient:nonNamespaced
// +k8s:deepcopy-gen:interfaces=k8s.io/apimachinery/pkg/runtime.Object

type App struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`

	Spec AppSpec `json:"spec,omitempty"`
}

// +k8s:deepcopy-gen:interfaces=k8s.io/apimachinery/pkg/runtime.Object

type AppList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`

	Items []App `json:"items"`
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

// +k8s:deepcopy-gen:interfaces=k8s.io/apimachinery/pkg/runtime.Object

type AppRolloutList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`

	Items []AppRollout `json:"items"`
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
	ReadyAssignments   int64                 `json:"readyAssignments"`
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
	AppRolloutConditionReady   AppRolloutConditionType = "Ready"
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

// +k8s:deepcopy-gen:interfaces=k8s.io/apimachinery/pkg/runtime.Object

type ChartAssignmentList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`

	Items []ChartAssignment `json:"items"`
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
}

type ChartAssignmentPhase string

const (
	// Accepted is set once the controller has observed the CA and started
	// taking action.
	ChartAssignmentPhaseAccepted ChartAssignmentPhase = "Accepted"
	// LoadingChart marks the begin of parsing the chart and its metadata.
	ChartAssignmentPhaseLoadingChart ChartAssignmentPhase = "LoadingChart"
	// TODO(ensonic): Installing is only used with 'helm'.
	ChartAssignmentPhaseInstalling ChartAssignmentPhase = "Installing"
	// TODO(ensonic): Updating is only used with 'synk'.
	ChartAssignmentPhaseUpdating ChartAssignmentPhase = "Updating"
	// Deleting is set when starting deletion to avoid flipping failure status
	// during turndown.
	ChartAssignmentPhaseDeleting ChartAssignmentPhase = "Deleting"
	// Settled status signals that all resources from the chart have been applied.
	ChartAssignmentPhaseSettled ChartAssignmentPhase = "Settled"
	// Deleted is set when the deletion phase finished.
	ChartAssignmentPhaseDeleted ChartAssignmentPhase = "Deleted"
	// Failed indicates that an error occured. The 'err' fields will give extra
	// information and 'retry' will indicate if the controller is trying to
	// recover from the error.
	ChartAssignmentPhaseFailed ChartAssignmentPhase = "Failed"
	// Ready status is set when all pods are running.
	// TODO(ensonic): check other resource readyness too?
	ChartAssignmentPhaseReady ChartAssignmentPhase = "Ready"
)

type ChartAssignmentCondition struct {
	Type               ChartAssignmentConditionType `json:"type"`
	Status             corev1.ConditionStatus       `json:"status"`
	LastUpdateTime     metav1.Time                  `json:"lastUpdateTime,omitempty"`
	LastTransitionTime metav1.Time                  `json:"lastTransitionTime,omitempty"`
	Message            string                       `json:"message,omitempty"`
}

type ChartAssignmentConditionType string

const (
	ChartAssignmentConditionSettled ChartAssignmentConditionType = "Settled"
	ChartAssignmentConditionReady   ChartAssignmentConditionType = "Ready"
)
