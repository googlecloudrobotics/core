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

package main

import (
	"os"

	registry "github.com/googlecloudrobotics/core/src/go/pkg/apis/registry/v1alpha1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/client-go/dynamic/fake"
	"net/http"
	"net/http/httptest"
	"testing"
)

func TestWaitForService_OkIfServiceResponds(t *testing.T) {
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {}))
	defer server.Close()

	err := waitForService(server.Client(), server.URL, 1)
	if err != nil {
		t.Errorf("waitForService returned error: %v", err)
	}
}

func TestParseKeyValues_ReturnsEmptyMapOnEmptyInput(t *testing.T) {
	_, err := parseKeyValues("")
	if err != nil {
		t.Errorf("Empty should be okay, but returned %v", err)
	}
}

func TestParseKeyValues_HandlesSingleEntry(t *testing.T) {
	l, err := parseKeyValues("foo=bar")
	if err != nil {
		t.Errorf("Failed to parse single entry, but returned %v", err)
	}
	v, ok := l["foo"]
	if !ok {
		t.Errorf("No 'foo' entry created")
	}
	if v != "bar" {
		t.Errorf("labels['foo'] should be 'bar', but is %q", v)
	}
}

func TestParseKeyValues_HandlesMultipleEntries(t *testing.T) {
	l, err := parseKeyValues("foo=bar,zoo=zar")
	if err != nil {
		t.Errorf("Failed to parse single entry, but returned %v", err)
	}
	v, ok := l["foo"]
	if !ok {
		t.Errorf("No 'foo' entry created")
	}
	if v != "bar" {
		t.Errorf("labels['foo'] should be 'bar', but is %q", v)
	}
	v, ok = l["zoo"]
	if !ok {
		t.Errorf("No 'zoo' entry created")
	}
	if v != "zar" {
		t.Errorf("labels['zoo'] should be 'zar', but is %q", v)
	}
}

func TestParseKeyValues_HandlesSpaces(t *testing.T) {
	l, err := parseKeyValues("foo=bar baz")
	if err != nil {
		t.Errorf("Failed to parse single entry, but returned %v", err)
	}
	v, ok := l["foo"]
	if !ok {
		t.Errorf("No 'foo' entry created")
	}
	if v != "bar baz" {
		t.Errorf("labels['foo'] should be 'bar baz', but is %q", v)
	}
}

func TestCreateOrUpdateRobot_Succeeds(t *testing.T) {
	hostname, err := os.Hostname()
	if err != nil {
		t.Fatal("Could not determine hostname")
	}
	os.Setenv("HOST_HOSTNAME", hostname)

	sc := runtime.NewScheme()
	registry.AddToScheme(sc)
	*robotName = "robot_name"

	tests := []struct {
		desc  string
		robot *registry.Robot
	}{
		{
			"other robot",
			&registry.Robot{
				ObjectMeta: metav1.ObjectMeta{
					Name:      "other_robot",
					Namespace: "default",
				},
			},
		},
		{
			"robot without label",
			&registry.Robot{
				ObjectMeta: metav1.ObjectMeta{
					Name:      *robotName,
					Namespace: "default",
				},
			},
		},
		{
			"robot with other label",
			&registry.Robot{
				ObjectMeta: metav1.ObjectMeta{
					Name:      *robotName,
					Namespace: "default",
					Labels:    map[string]string{"cloudrobotics.com/ssh-port": "22"},
				},
			},
		},
		{
			"robot with same hostname",
			&registry.Robot{
				ObjectMeta: metav1.ObjectMeta{
					Name:      *robotName,
					Namespace: "default",
					Labels:    map[string]string{"cloudrobotics.com/master-host": hostname},
				},
			},
		},
		// This should ask user to confirm or Ctrl+C the app. We expect
		// success in a test environment where stdin is empty.
		{
			"robot with different hostname",
			&registry.Robot{
				ObjectMeta: metav1.ObjectMeta{
					Name:      *robotName,
					Namespace: "default",
					Labels:    map[string]string{"cloudrobotics.com/master-host": "other-host"},
				},
			},
		},
	}
	for _, tc := range tests {
		t.Run(tc.desc, func(t *testing.T) {
			c := fake.NewSimpleDynamicClient(sc, tc.robot)
			labels := map[string]string{}
			annotations := map[string]string{}
			if err := createOrUpdateRobot(c, labels, annotations); err != nil {
				t.Errorf("createOrUpdateRobot() failed unexpectedly:  %v", err)
			}
		})
	}
}
