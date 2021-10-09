//go:build !ignore_autogenerated
// +build !ignore_autogenerated

// Copyright 2021 The Cloud Robotics Authors
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

// Code generated by deepcopy-gen. DO NOT EDIT.

package v1alpha1

import (
	runtime "k8s.io/apimachinery/pkg/runtime"
)

// DeepCopyInto is an autogenerated deepcopy function, copying the receiver, writing into out. in must be non-nil.
func (in *Robot) DeepCopyInto(out *Robot) {
	*out = *in
	out.TypeMeta = in.TypeMeta
	in.ObjectMeta.DeepCopyInto(&out.ObjectMeta)
	out.Spec = in.Spec
	in.Status.DeepCopyInto(&out.Status)
	return
}

// DeepCopy is an autogenerated deepcopy function, copying the receiver, creating a new Robot.
func (in *Robot) DeepCopy() *Robot {
	if in == nil {
		return nil
	}
	out := new(Robot)
	in.DeepCopyInto(out)
	return out
}

// DeepCopyObject is an autogenerated deepcopy function, copying the receiver, creating a new runtime.Object.
func (in *Robot) DeepCopyObject() runtime.Object {
	if c := in.DeepCopy(); c != nil {
		return c
	}
	return nil
}

// DeepCopyInto is an autogenerated deepcopy function, copying the receiver, writing into out. in must be non-nil.
func (in *RobotConfiguration) DeepCopyInto(out *RobotConfiguration) {
	*out = *in
	return
}

// DeepCopy is an autogenerated deepcopy function, copying the receiver, creating a new RobotConfiguration.
func (in *RobotConfiguration) DeepCopy() *RobotConfiguration {
	if in == nil {
		return nil
	}
	out := new(RobotConfiguration)
	in.DeepCopyInto(out)
	return out
}

// DeepCopyInto is an autogenerated deepcopy function, copying the receiver, writing into out. in must be non-nil.
func (in *RobotList) DeepCopyInto(out *RobotList) {
	*out = *in
	out.TypeMeta = in.TypeMeta
	in.ListMeta.DeepCopyInto(&out.ListMeta)
	if in.Items != nil {
		in, out := &in.Items, &out.Items
		*out = make([]Robot, len(*in))
		for i := range *in {
			(*in)[i].DeepCopyInto(&(*out)[i])
		}
	}
	return
}

// DeepCopy is an autogenerated deepcopy function, copying the receiver, creating a new RobotList.
func (in *RobotList) DeepCopy() *RobotList {
	if in == nil {
		return nil
	}
	out := new(RobotList)
	in.DeepCopyInto(out)
	return out
}

// DeepCopyObject is an autogenerated deepcopy function, copying the receiver, creating a new runtime.Object.
func (in *RobotList) DeepCopyObject() runtime.Object {
	if c := in.DeepCopy(); c != nil {
		return c
	}
	return nil
}

// DeepCopyInto is an autogenerated deepcopy function, copying the receiver, writing into out. in must be non-nil.
func (in *RobotSpec) DeepCopyInto(out *RobotSpec) {
	*out = *in
	return
}

// DeepCopy is an autogenerated deepcopy function, copying the receiver, creating a new RobotSpec.
func (in *RobotSpec) DeepCopy() *RobotSpec {
	if in == nil {
		return nil
	}
	out := new(RobotSpec)
	in.DeepCopyInto(out)
	return out
}

// DeepCopyInto is an autogenerated deepcopy function, copying the receiver, writing into out. in must be non-nil.
func (in *RobotStatus) DeepCopyInto(out *RobotStatus) {
	*out = *in
	out.Cloud = in.Cloud
	in.Robot.DeepCopyInto(&out.Robot)
	out.Configuration = in.Configuration
	return
}

// DeepCopy is an autogenerated deepcopy function, copying the receiver, creating a new RobotStatus.
func (in *RobotStatus) DeepCopy() *RobotStatus {
	if in == nil {
		return nil
	}
	out := new(RobotStatus)
	in.DeepCopyInto(out)
	return out
}

// DeepCopyInto is an autogenerated deepcopy function, copying the receiver, writing into out. in must be non-nil.
func (in *RobotStatusCloud) DeepCopyInto(out *RobotStatusCloud) {
	*out = *in
	return
}

// DeepCopy is an autogenerated deepcopy function, copying the receiver, creating a new RobotStatusCloud.
func (in *RobotStatusCloud) DeepCopy() *RobotStatusCloud {
	if in == nil {
		return nil
	}
	out := new(RobotStatusCloud)
	in.DeepCopyInto(out)
	return out
}

// DeepCopyInto is an autogenerated deepcopy function, copying the receiver, writing into out. in must be non-nil.
func (in *RobotStatusRobot) DeepCopyInto(out *RobotStatusRobot) {
	*out = *in
	in.UpdateTime.DeepCopyInto(&out.UpdateTime)
	in.LastStateChange.DeepCopyInto(&out.LastStateChange)
	return
}

// DeepCopy is an autogenerated deepcopy function, copying the receiver, creating a new RobotStatusRobot.
func (in *RobotStatusRobot) DeepCopy() *RobotStatusRobot {
	if in == nil {
		return nil
	}
	out := new(RobotStatusRobot)
	in.DeepCopyInto(out)
	return out
}
