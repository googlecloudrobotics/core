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

package setup

import (
	"bytes"
	"crypto/rand"
	"crypto/rsa"
	"crypto/x509"
	"encoding/pem"
	"io"
	"net/http"
	"net/http/httptest"
	"reflect"
	"strings"
	"testing"

	"github.com/googlecloudrobotics/core/src/go/pkg/robotauth"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/client-go/dynamic/fake"
)

func TestSelectRobot(t *testing.T) {
	robots := []unstructured.Unstructured{
		{
			Object: map[string]interface{}{
				"apiVersion": "registry.cloudrobotics.com/v1alpha1",
				"kind":       "Robot",
				"metadata": map[string]interface{}{
					"namespace": "default",
					"name":      "ro-1234",
					"labels": map[string]interface{}{
						"cloudrobotics.com/robot-name": "ro-1234",
					},
				},
				"spec": map[string]interface{}{
					"type": "test",
				},
			},
		},
	}

	reader := strings.NewReader("1\n")

	id, err := selectRobot(reader, robots)
	if id != "ro-1234" || err != nil {
		t.Errorf("selectRobot(reader, oneRobot) = %v, %v want ro-1234, nil", id, err)
	}
}

func TestWaitForService_OkIfServiceResponds(t *testing.T) {
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {}))
	defer server.Close()

	err := WaitForService(server.Client(), server.URL, 1)
	if err != nil {
		t.Errorf("WaitForService returned error: %v", err)
	}
}

func TestCreateOrUpdateRobot(t *testing.T) {
	scheme := runtime.NewScheme()
	client := fake.NewSimpleDynamicClient(scheme)
	resource := client.Resource(schema.GroupVersionResource{
		Group:    "registry.cloudrobotics.com",
		Version:  "v1alpha1",
		Resource: "robots",
	})

	ctx := t.Context()
	robotName := "test-robot"
	robotType := "test-type"
	project := "test-project"
	labels := map[string]string{"foo": "bar"}
	annotations := map[string]string{"baz": "qux"}

	// 1. Test Create
	err := CreateOrUpdateRobot(ctx, resource, robotName, robotType, project, labels, annotations)
	if err != nil {
		t.Fatalf("CreateOrUpdateRobot failed to create: %v", err)
	}

	got, err := resource.Get(ctx, robotName, metav1.GetOptions{})
	if err != nil {
		t.Fatalf("Failed to get created robot: %v", err)
	}
	if got.GetName() != robotName {
		t.Errorf("Expected name %q, got %q", robotName, got.GetName())
	}
	if !reflect.DeepEqual(got.GetLabels(), labels) {
		t.Errorf("Expected labels %v, got %v", labels, got.GetLabels())
	}
	if !reflect.DeepEqual(got.GetAnnotations(), annotations) {
		t.Errorf("Expected annotations %v, got %v", annotations, got.GetAnnotations())
	}

	// 2. Test Update
	newLabels := map[string]string{"foo": "updated", "new": "label"}
	err = CreateOrUpdateRobot(ctx, resource, robotName, "new-type", project, newLabels, nil)
	if err != nil {
		t.Fatalf("CreateOrUpdateRobot failed to update: %v", err)
	}

	got, err = resource.Get(ctx, robotName, metav1.GetOptions{})
	if err != nil {
		t.Fatalf("Failed to get updated robot: %v", err)
	}
	if !reflect.DeepEqual(got.GetLabels(), newLabels) {
		t.Errorf("Expected labels %v, got %v", newLabels, got.GetLabels())
	}
	spec := got.Object["spec"].(map[string]interface{})
	if spec["type"] != "new-type" {
		t.Errorf("Expected type %q, got %q", "new-type", spec["type"])
	}
}

func TestGetPublicKey(t *testing.T) {
	key, err := rsa.GenerateKey(rand.Reader, 2048)
	if err != nil {
		t.Fatal(err)
	}

	pkcs8Bytes, err := x509.MarshalPKCS8PrivateKey(key)
	if err != nil {
		t.Fatal(err)
	}

	tests := []struct {
		name string
		pem  []byte
	}{
		{
			name: "PKCS1",
			pem: pem.EncodeToMemory(&pem.Block{
				Type:  "RSA PRIVATE KEY",
				Bytes: x509.MarshalPKCS1PrivateKey(key),
			}),
		},
		{
			name: "PKCS8",
			pem: pem.EncodeToMemory(&pem.Block{
				Type:  "PRIVATE KEY",
				Bytes: pkcs8Bytes,
			}),
		},
	}

	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			pub, err := getPublicKey(tc.pem)
			if err != nil {
				t.Fatalf("getPublicKey() failed: %v", err)
			}
			if !bytes.Contains(pub, []byte("BEGIN PUBLIC KEY")) {
				t.Errorf("result missing header: %s", string(pub))
			}
		})
	}
}

func TestGetPublicKey_Invalid(t *testing.T) {
	tests := []struct {
		name string
		pem  []byte
	}{
		{
			name: "not a PEM block",
			pem:  []byte("not a key"),
		},
		{
			name: "empty input",
			pem:  []byte(""),
		},
	}

	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			if _, err := getPublicKey(tc.pem); err == nil {
				t.Errorf("getPublicKey(%s) should have failed", tc.name)
			}
		})
	}
}

func TestGetRobotName(t *testing.T) {
	scheme := runtime.NewScheme()
	client := fake.NewSimpleDynamicClient(scheme)
	resource := client.Resource(schema.GroupVersionResource{
		Group:    "registry.cloudrobotics.com",
		Version:  "v1alpha1",
		Resource: "robots",
	})

	robot := &unstructured.Unstructured{}
	robot.SetKind("Robot")
	robot.SetAPIVersion("registry.cloudrobotics.com/v1alpha1")
	robot.SetName("my-robot")
	_, err := resource.Create(t.Context(), robot, metav1.CreateOptions{})
	if err != nil {
		t.Fatal(err)
	}

	tests := []struct {
		name      string
		robotName string
		wantName  string
		wantErr   bool
		errSubstr string
	}{
		{
			name:      "exists",
			robotName: "my-robot",
			wantName:  "my-robot",
		},
		{
			name:      "not found",
			robotName: "non-existent",
			wantErr:   true,
			errSubstr: "robot non-existent not found",
		},
	}

	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			name, err := GetRobotName(t.Context(), nil, resource, tc.robotName)
			if (err != nil) != tc.wantErr {
				t.Fatalf("GetRobotName() error = %v, wantErr %v", err, tc.wantErr)
			}
			if tc.wantErr {
				if !strings.Contains(err.Error(), tc.errSubstr) {
					t.Errorf("Expected error to contain %q, got %q", tc.errSubstr, err.Error())
				}
			} else if name != tc.wantName {
				t.Errorf("GetRobotName() = %q, want %q", name, tc.wantName)
			}
		})
	}
}

func TestPublishCredentialsToCloud_Success(t *testing.T) {
	key, err := rsa.GenerateKey(rand.Reader, 2048)
	if err != nil {
		t.Fatal(err)
	}
	privateKeyPEM := pem.EncodeToMemory(&pem.Block{
		Type:  "RSA PRIVATE KEY",
		Bytes: x509.MarshalPKCS1PrivateKey(key),
	})

	server := httptest.NewTLSServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.Method == http.MethodHead && r.URL.Path == "/apis/core.token-vendor/v1/public-key.read" {
			w.WriteHeader(http.StatusOK)
			return
		}
		if r.Method == http.MethodPost && r.URL.Path == "/apis/core.token-vendor/v1/public-key.publish" {
			if r.URL.Query().Get("device-id") != "my-device" {
				w.WriteHeader(http.StatusBadRequest)
				return
			}
			body, err := io.ReadAll(r.Body)
			if err != nil {
				w.WriteHeader(http.StatusInternalServerError)
				return
			}
			if !bytes.Contains(body, []byte("BEGIN PUBLIC KEY")) {
				w.WriteHeader(http.StatusBadRequest)
				w.Write([]byte("expected public key"))
				return
			}
			w.WriteHeader(http.StatusOK)
			return
		}
		w.WriteHeader(http.StatusNotFound)
	}))
	defer server.Close()

	domain := strings.TrimPrefix(server.URL, "https://")

	auth := &robotauth.RobotAuth{
		Domain:               domain,
		PrivateKey:           privateKeyPEM,
		PublicKeyRegistryId: "my-device",
	}

	err = PublishCredentialsToCloud(server.Client(), auth, 1)
	if err != nil {
		t.Fatalf("PublishCredentialsToCloud failed: %v", err)
	}
}

