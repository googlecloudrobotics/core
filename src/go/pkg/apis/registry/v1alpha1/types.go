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
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
)

// +genclient
// +k8s:deepcopy-gen:interfaces=k8s.io/apimachinery/pkg/runtime.Object

type Robot struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`

	Spec   RobotSpec   `json:"spec,omitempty"`
	Status RobotStatus `json:"status,omitempty"`
}

// +k8s:deepcopy-gen:interfaces=k8s.io/apimachinery/pkg/runtime.Object

type RobotList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`

	Items []Robot `json:"items"`
}

type RobotSpec struct {
	Type    string `json:"type,omitempty"`
	Project string `json:"project,omitempty"`
}

type RobotStatus struct {
	Cloud         RobotStatusCloud   `json:"cloud,omitempty"`
	Robot         RobotStatusRobot   `json:"robot,omitempty"`
	Configuration RobotConfiguration `json:"configuration,omitempty"`
}

type RobotStatusCloud struct {
}

type RobotStatusRobot struct {
	UpdateTime                 metav1.Time `json:"updateTime,omitempty"`
	State                      RobotState  `json:"state,omitempty"`
	LastStateChange            metav1.Time `json:"lastStateChangeTime,omitempty"`
	BatteryPercentage          float64     `json:"batteryPercentage,omitempty"`
	EmergencyStopButtonPressed bool        `json:"emergencyStopButtonPressed,omitempty"`
}

type RobotConfiguration struct {
	TrolleyAttached bool `json:"trolleyAttached,omitempty"`
}

type RobotState string

const (
	RobotStateUndefined     RobotState = "UNDEFINED"
	RobotStateUnavailable              = "UNAVAILABLE"
	RobotStateAvailable                = "AVAILABLE"
	RobotStateEmergencyStop            = "EMERGENCY_STOP"
	RobotStateError                    = "ERROR"
)
