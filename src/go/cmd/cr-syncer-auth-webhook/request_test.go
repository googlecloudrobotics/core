package main

import (
	"testing"

	"github.com/google/go-cmp/cmp"
)

func TestParseURL(t *testing.T) {
	tests := []struct {
		desc string
		url  string
		want incomingRequest
	}{
		{
			desc: "watch request, filtered",
			url:  "http://host/apis/core.kubernetes/apis/apps.cloudrobotics.com/v1alpha1/chartassignments?labelSelector=cloudrobotics.com%2Frobot-name%3Dmy-robot",
			want: incomingRequest{
				GroupKind: "apps.cloudrobotics.com/chartassignments",
				RobotName: "my-robot",
			},
		},
		{
			desc: "watch request, unfiltered",
			url:  "http://host/apis/core.kubernetes/apis/registry.cloudrobotics.com/v1alpha1/robottypes",
			want: incomingRequest{
				GroupKind: "registry.cloudrobotics.com/robottypes",
			},
		},
		{
			desc: "watch request, with namespace",
			url:  "http://host/apis/core.kubernetes/apis/apps.cloudrobotics.com/v1alpha1/namespaces/default/chartassignments?labelSelector=cloudrobotics.com%2Frobot-name%3Dmy-robot",
			want: incomingRequest{
				GroupKind: "apps.cloudrobotics.com/chartassignments",
				RobotName: "my-robot",
			},
		},
		{
			desc: "get request",
			url:  "http://host/apis/core.kubernetes/apis/apps.cloudrobotics.com/v1alpha1/namespaces/default/chartassignments/resource-for-my-robot",
			want: incomingRequest{
				GroupKind:    "apps.cloudrobotics.com/chartassignments",
				ResourceName: "resource-for-my-robot",
			},
		},
		{
			desc: "status post request, with namespace",
			url:  "http://host/apis/core.kubernetes/apis/apps.cloudrobotics.com/v1alpha1/namespaces/default/chartassignments/resource-for-my-robot/status",
			want: incomingRequest{
				GroupKind:    "apps.cloudrobotics.com/chartassignments",
				ResourceName: "resource-for-my-robot",
			},
		},
		{
			desc: "status post request, without namespace",
			url:  "http://host/apis/core.kubernetes/apis/apps.cloudrobotics.com/v1alpha1/chartassignments/resource-for-my-robot/status?timeout=5m5s",
			want: incomingRequest{
				GroupKind:    "apps.cloudrobotics.com/chartassignments",
				ResourceName: "resource-for-my-robot",
			},
		},
	}

	for _, tc := range tests {
		t.Run(tc.desc, func(t *testing.T) {
			got, err := parseURL(tc.url)
			if err != nil {
				t.Fatalf("parseURL(%q) returned error: %v", tc.url, err)
			}
			if diff := cmp.Diff(tc.want, *got); diff != "" {
				t.Errorf("parseURL(%q) returned diff (-want +got):\n%s", tc.url, diff)
			}
		})
	}
}
func TestParseURLErrors(t *testing.T) {
	tests := []struct {
		desc string
		url  string
	}{
		{
			desc: "empty robot name",
			url:  "http://host/apis/core.kubernetes/apis/apps.cloudrobotics.com/v1alpha1/chartassignments?labelSelector=cloudrobotics.com%2Frobot-name%3D",
		},
		{
			desc: "over-broad label selector: robot-name!=my-robot",
			url:  "http://host/apis/core.kubernetes/apis/apps.cloudrobotics.com/v1alpha1/chartassignments?labelSelector=cloudrobotics.com%2Frobot-name%21%3Dmy-robot",
		},
		{
			desc: "core API (not a CR)",
			url:  "http://host/apis/core.kubernetes/api/v1/namespaces/default/pods/cr-syncer-6676b4958d-p9hqw",
		},
	}

	for _, tc := range tests {
		t.Run(tc.desc, func(t *testing.T) {
			_, err := parseURL(tc.url)
			if err == nil {
				t.Fatalf("parseURL(%q) succeeded unexpected", tc.url)
			}
		})
	}
}
