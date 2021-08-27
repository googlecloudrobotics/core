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
	b64 "encoding/base64"
	"net/http"
	"net/http/httptest"
	"os"
	"reflect"
	"testing"

	registry "github.com/googlecloudrobotics/core/src/go/pkg/apis/registry/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	k8serrors "k8s.io/apimachinery/pkg/api/errors"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"k8s.io/apimachinery/pkg/runtime"
	dynfake "k8s.io/client-go/dynamic/fake"
	corefake "k8s.io/client-go/kubernetes/fake"
	k8stest "k8s.io/client-go/testing"
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

func TestParseKeyValues_HandlesEscapedCommas(t *testing.T) {
	l, err := parseKeyValues("foo=bar\\,baz,zoo=zar")
	if err != nil {
		t.Errorf("Failed to parse single entry, but returned %v", err)
	}
	v, ok := l["foo"]
	if !ok {
		t.Errorf("No 'foo' entry created")
	}
	if v != "bar,baz" {
		t.Errorf("labels['foo'] should be 'bar,baz', but is %q", v)
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

func TestCheckRobotName_SucceedsWhenCRDNotFound(t *testing.T) {
	sc := runtime.NewScheme()
	*robotName = "robot_name"

	c := dynfake.NewSimpleDynamicClient(sc)
	// In a fresh cluster, the Robot CRD doesn't exist, so GET robots
	// returns a 404.
	c.PrependReactor("list", "robots", func(k8stest.Action) (bool, runtime.Object, error) {
		return true, nil, &k8serrors.StatusError{metav1.Status{
			Status:  metav1.StatusFailure,
			Code:    http.StatusNotFound,
			Reason:  metav1.StatusReasonNotFound,
			Message: "the server could not find the requested resource",
		}}
	})
	err := checkRobotName(c)
	if err != nil {
		t.Errorf("checkRobotName() failed unexpectedly: %v", err)
	}
}

func TestCheckRobotName(t *testing.T) {
	sc := runtime.NewScheme()
	registry.AddToScheme(sc)
	*robotName = "robot_name"

	tests := []struct {
		desc      string
		robots    []runtime.Object
		wantError bool
	}{
		{
			desc:      "empty cluster",
			robots:    []runtime.Object{},
			wantError: false,
		},
		{
			desc: "robot with same name",
			robots: []runtime.Object{
				&unstructured.Unstructured{
					Object: map[string]interface{}{
						"apiVersion": "registry.cloudrobotics.com/v1alpha1",
						"kind":       "Robot",
						"metadata": map[string]interface{}{
							"name":      *robotName,
							"namespace": "default",
						},
					},
				},
			},
			wantError: false,
		},
		{
			desc: "robot with other name",
			robots: []runtime.Object{
				&unstructured.Unstructured{
					Object: map[string]interface{}{
						"apiVersion": "registry.cloudrobotics.com/v1alpha1",
						"kind":       "Robot",
						"metadata": map[string]interface{}{
							"name":      "other_name",
							"namespace": "default",
						},
					},
				},
			},
			wantError: true,
		},
	}
	for _, tc := range tests {
		t.Run(tc.desc, func(t *testing.T) {
			c := dynfake.NewSimpleDynamicClient(sc, tc.robots...)
			err := checkRobotName(c)
			if tc.wantError && err == nil {
				t.Errorf("checkRobotName() succeeded unexpectedly")
			}
			if !tc.wantError && err != nil {
				t.Errorf("checkRobotName() failed unexpectedly: %v", err)
			}
		})
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
		desc       string
		labels     map[string]string
		robot      *registry.Robot
		wantLabels map[string]string
	}{
		{
			"other robot",
			map[string]string{},
			&registry.Robot{
				ObjectMeta: metav1.ObjectMeta{
					Name:      "other_robot",
					Namespace: "default",
				},
			},
			map[string]string{
				"cloudrobotics.com/master-host": hostname,
				"cloudrobotics.com/robot-name":  "robot_name",
			},
		},
		{
			"robot without label",
			map[string]string{},
			&registry.Robot{
				ObjectMeta: metav1.ObjectMeta{
					Name:      *robotName,
					Namespace: "default",
				},
			},
			map[string]string{
				"cloudrobotics.com/master-host": hostname,
				"cloudrobotics.com/robot-name":  "robot_name",
			},
		},
		{
			"robot with other label",
			map[string]string{},
			&registry.Robot{
				ObjectMeta: metav1.ObjectMeta{
					Name:      *robotName,
					Namespace: "default",
					Labels:    map[string]string{"cloudrobotics.com/ssh-port": "22"},
				},
			},
			map[string]string{
				"cloudrobotics.com/master-host": hostname,
				"cloudrobotics.com/robot-name":  "robot_name",
				"cloudrobotics.com/ssh-port":    "22",
			},
		},
		{
			"robot with same hostname",
			map[string]string{},
			&registry.Robot{
				ObjectMeta: metav1.ObjectMeta{
					Name:      *robotName,
					Namespace: "default",
					Labels:    map[string]string{"cloudrobotics.com/master-host": hostname},
				},
			},
			map[string]string{
				"cloudrobotics.com/master-host": hostname,
				"cloudrobotics.com/robot-name":  "robot_name",
			},
		},
		{
			"robot with different hostname",
			map[string]string{},
			&registry.Robot{
				ObjectMeta: metav1.ObjectMeta{
					Name:      *robotName,
					Namespace: "default",
					Labels:    map[string]string{"cloudrobotics.com/master-host": "other-host"},
				},
			},
			map[string]string{
				"cloudrobotics.com/master-host": hostname,
				"cloudrobotics.com/robot-name":  "robot_name",
			},
		},
		{
			"master-host given as input",
			map[string]string{
				"cloudrobotics.com/master-host": "correct-host",
			},
			&registry.Robot{
				ObjectMeta: metav1.ObjectMeta{
					Name:      *robotName,
					Namespace: "default",
					Labels:    map[string]string{"cloudrobotics.com/master-host": "other-host"},
				},
			},
			map[string]string{
				"cloudrobotics.com/master-host": "correct-host",
				"cloudrobotics.com/robot-name":  "robot_name",
			},
		},
	}
	for _, tc := range tests {
		t.Run(tc.desc, func(t *testing.T) {
			c := dynfake.NewSimpleDynamicClient(sc, tc.robot)
			annotations := map[string]string{}
			if err := createOrUpdateRobot(c, tc.labels, annotations); err != nil {
				t.Fatalf("createOrUpdateRobot() failed unexpectedly:  %v", err)
			}

			robotClient := c.Resource(robotGVR).Namespace("default")
			robot, err := robotClient.Get(*robotName, metav1.GetOptions{})
			if err != nil {
				t.Fatalf("Failed getting robot: %v", err)
			}
			got, ok, err := unstructured.NestedStringMap(robot.Object, "metadata", "labels")
			if err != nil {
				t.Fatalf("failed parsing robot labels: %v", err)
			}
			if !ok {
				t.Fatalf("robot %q is missing the master host label", *robotName)
			}
			if !reflect.DeepEqual(got, tc.wantLabels) {
				t.Errorf("labels:\n%q\nwant:\n%q", got, tc.wantLabels)
			}
		})
	}
}

func TestEnsureWebhookCerts_DoesNotReplaceCerts(t *testing.T) {
	wantCert := "1234"
	wantKey := "abcd"
	c := corefake.NewSimpleClientset(
		&corev1.Secret{
			ObjectMeta: metav1.ObjectMeta{
				Name:      "robot-master-tls",
				Namespace: "default",
				Labels:    map[string]string{"cert-format": "v2"},
			},
			Data: map[string][]byte{
				"tls.crt": []byte(wantCert),
				"tls.key": []byte(wantKey),
			},
		},
	)

	encCert, encKey, err := ensureWebhookCerts(c, "default")
	if err != nil {
		t.Fatalf("failed getting cert and keys: %v", err)
	}
	cert, err := b64.URLEncoding.DecodeString(encCert)
	if err != nil {
		t.Fatalf("failed decoding cert: %v", err)
	}
	if string(cert) != wantCert {
		t.Fatalf("cert is %q, expected %q", cert, wantCert)
	}
	key, err := b64.URLEncoding.DecodeString(encKey)
	if err != nil {
		t.Fatalf("failed decoding cert: %v", err)
	}
	if string(key) != wantKey {
		t.Fatalf("key is %q, expected %q", cert, wantKey)
	}
}
