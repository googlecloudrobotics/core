package main

import (
	"net/http"
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
		{
			desc: "invalid short namespaced path",
			url:  "http://host/apis/core.kubernetes/apis/apps.cloudrobotics.com/v1alpha1/namespaces",
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

func TestExtractSubrequestURL(t *testing.T) {
	tests := []struct {
		desc        string
		originalURL string
		envoyPath   string
		want        string
		wantErr     bool
	}{
		{
			desc:        "extracts X-Original-Url when present",
			originalURL: "http://host/apis/core.kubernetes/apis/apps/v1/deployments",
			envoyPath:   "",
			want:        "http://host/apis/core.kubernetes/apis/apps/v1/deployments",
		},
		{
			desc:        "extracts X-Envoy-Original-Path when present",
			originalURL: "",
			envoyPath:   "/apis/core.kubernetes/apis/apps/v1/deployments",
			want:        "/apis/core.kubernetes/apis/apps/v1/deployments",
		},
		{
			desc:        "rejects ambiguous dual headers",
			originalURL: "http://host/apis/core.kubernetes/apis/apps/v1/deployments",
			envoyPath:   "/apis/core.kubernetes/apis/apps/v1/deployments",
			wantErr:     true,
		},
		{
			desc:        "rejects missing subrequest headers",
			originalURL: "",
			envoyPath:   "",
			wantErr:     true,
		},
	}

	for _, tc := range tests {
		t.Run(tc.desc, func(t *testing.T) {
			req, err := http.NewRequest(http.MethodGet, "/webhook", nil)
			if err != nil {
				t.Fatalf("http.NewRequest failed: %v", err)
			}
			if tc.originalURL != "" {
				req.Header.Set("X-Original-Url", tc.originalURL)
			}
			if tc.envoyPath != "" {
				req.Header.Set("X-Envoy-Original-Path", tc.envoyPath)
			}
			got, err := extractSubrequestURL(req)
			if tc.wantErr {
				if err == nil {
					t.Errorf("extractSubrequestURL() succeeded, want error")
				}
				return
			}
			if err != nil {
				t.Fatalf("extractSubrequestURL() unexpected error: %v", err)
			}
			if got != tc.want {
				t.Errorf("extractSubrequestURL() = %q, want %q", got, tc.want)
			}
		})
	}
}
