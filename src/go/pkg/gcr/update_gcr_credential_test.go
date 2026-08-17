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

package gcr

import (
	"encoding/json"
	"fmt"
	"reflect"
	"testing"

	"golang.org/x/oauth2"
	corev1 "k8s.io/api/core/v1"
	k8serrors "k8s.io/apimachinery/pkg/api/errors"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/client-go/kubernetes/fake"
	clienttesting "k8s.io/client-go/testing"
)

func TestDockercfgJSON(t *testing.T) {
	expectedJSON := `{
  "https://gcr.io":{"username":"oauth2accesstoken","password":"ya29.yaddayadda","email":"not@val.id","auth":"b2F1dGgyYWNjZXNzdG9rZW46eWEyOS55YWRkYXlhZGRh"},
  "https://asia.gcr.io":{"username":"oauth2accesstoken","password":"ya29.yaddayadda","email":"not@val.id","auth":"b2F1dGgyYWNjZXNzdG9rZW46eWEyOS55YWRkYXlhZGRh"},
  "https://eu.gcr.io":{"username":"oauth2accesstoken","password":"ya29.yaddayadda","email":"not@val.id","auth":"b2F1dGgyYWNjZXNzdG9rZW46eWEyOS55YWRkYXlhZGRh"},
  "https://us.gcr.io":{"username":"oauth2accesstoken","password":"ya29.yaddayadda","email":"not@val.id","auth":"b2F1dGgyYWNjZXNzdG9rZW46eWEyOS55YWRkYXlhZGRh"}
}`

	gotJSON := DockerCfgJSON("ya29.yaddayadda")

	var expected, got map[string]interface{}
	if err := json.Unmarshal([]byte(expectedJSON), &expected); err != nil {
		t.Fatalf("failed to unmarshal expected JSON: %v", err)
	}
	if err := json.Unmarshal(gotJSON, &got); err != nil {
		t.Fatalf("failed to unmarshal got JSON: %v", err)
	}

	if !reflect.DeepEqual(expected, got) {
		t.Errorf("JSONs do not match.\nExpected: %s\nGot: %s", expectedJSON, string(gotJSON))
	}
}

type fakeTokenSource struct {
	token *oauth2.Token
	err   error
}

func (ts *fakeTokenSource) Token() (*oauth2.Token, error) {
	return ts.token, ts.err
}

func TestUpdateGcrCredentials(t *testing.T) {
	deletionTime := metav1.Now()

	tests := []struct {
		name                  string
		tokenSource           oauth2.TokenSource
		initObjects           []runtime.Object
		reactor               func(*fake.Clientset)
		wantErr               bool
		wantSecretNS          []string // namespaces where we expect SecretName to exist and be updated
		wantPatchedNS         []string // namespaces where we expect default SA to be patched
		dontWantSecretNS      []string // namespaces where we expect SecretName to NOT exist at all
		wantUnchangedSecretNS []string // namespaces where we expect SecretName to exist but NOT be updated
	}{
		{
			name: "success_basic",
			tokenSource: &fakeTokenSource{
				token: &oauth2.Token{AccessToken: "fake-token"},
			},
			initObjects: []runtime.Object{
				&corev1.ServiceAccount{ObjectMeta: metav1.ObjectMeta{Name: "default", Namespace: "default"}},
				&corev1.Namespace{ObjectMeta: metav1.ObjectMeta{Name: "kube-system"}},
				&corev1.ServiceAccount{ObjectMeta: metav1.ObjectMeta{Name: "default", Namespace: "kube-system"}},
				&corev1.Namespace{ObjectMeta: metav1.ObjectMeta{Name: "other-ns"}}, // Should be skipped (no secret, not kube-system)
				&corev1.Namespace{ObjectMeta: metav1.ObjectMeta{Name: "ns-with-secret"}},
				&corev1.Secret{ObjectMeta: metav1.ObjectMeta{Name: SecretName, Namespace: "ns-with-secret"}},
				&corev1.ServiceAccount{ObjectMeta: metav1.ObjectMeta{Name: "default", Namespace: "ns-with-secret"}},
				&corev1.Namespace{ObjectMeta: metav1.ObjectMeta{Name: "deleting-ns", DeletionTimestamp: &deletionTime}},
				&corev1.Secret{ObjectMeta: metav1.ObjectMeta{Name: SecretName, Namespace: "deleting-ns"}}, // Even if it has secret, it should be skipped
			},
			wantSecretNS:          []string{"default", "kube-system", "ns-with-secret"},
			wantPatchedNS:         []string{"default", "kube-system", "ns-with-secret"},
			dontWantSecretNS:      []string{"other-ns"},
			wantUnchangedSecretNS: []string{"deleting-ns"},
		},
		{
			name: "token_error",
			tokenSource: &fakeTokenSource{
				err: fmt.Errorf("token error"),
			},
			wantErr: true,
		},
		{
			name: "default_secret_update_fail",
			tokenSource: &fakeTokenSource{
				token: &oauth2.Token{AccessToken: "fake-token"},
			},
			initObjects: []runtime.Object{
				&corev1.ServiceAccount{ObjectMeta: metav1.ObjectMeta{Name: "default", Namespace: "default"}},
			},
			reactor: func(c *fake.Clientset) {
				c.PrependReactor("create", "secrets", func(action clienttesting.Action) (handled bool, ret runtime.Object, err error) {
					createAction := action.(clienttesting.CreateAction)
					obj := createAction.GetObject().(*corev1.Secret)
					if obj.Namespace == "default" && obj.Name == SecretName {
						return true, nil, fmt.Errorf("fake default secret create error")
					}
					return false, nil, nil
				})
			},
			wantErr: true,
		},
		{
			name: "default_sa_patch_fail",
			tokenSource: &fakeTokenSource{
				token: &oauth2.Token{AccessToken: "fake-token"},
			},
			initObjects: []runtime.Object{
				&corev1.ServiceAccount{ObjectMeta: metav1.ObjectMeta{Name: "default", Namespace: "default"}},
			},
			reactor: func(c *fake.Clientset) {
				c.PrependReactor("patch", "serviceaccounts", func(action clienttesting.Action) (handled bool, ret runtime.Object, err error) {
					patchAction := action.(clienttesting.PatchAction)
					if patchAction.GetNamespace() == "default" && patchAction.GetName() == "default" {
						return true, nil, fmt.Errorf("fake default sa patch error")
					}
					return false, nil, nil
				})
			},
			wantErr: true,
		},
		{
			name: "list_namespaces_fail",
			tokenSource: &fakeTokenSource{
				token: &oauth2.Token{AccessToken: "fake-token"},
			},
			initObjects: []runtime.Object{
				&corev1.ServiceAccount{ObjectMeta: metav1.ObjectMeta{Name: "default", Namespace: "default"}},
			},
			reactor: func(c *fake.Clientset) {
				c.PrependReactor("list", "namespaces", func(action clienttesting.Action) (handled bool, ret runtime.Object, err error) {
					return true, nil, fmt.Errorf("fake list namespaces error")
				})
			},
			wantErr: true,
		},
		{
			name: "other_ns_secret_update_fail",
			tokenSource: &fakeTokenSource{
				token: &oauth2.Token{AccessToken: "fake-token"},
			},
			initObjects: []runtime.Object{
				&corev1.ServiceAccount{ObjectMeta: metav1.ObjectMeta{Name: "default", Namespace: "default"}},
				&corev1.Namespace{ObjectMeta: metav1.ObjectMeta{Name: "kube-system"}},
				&corev1.ServiceAccount{ObjectMeta: metav1.ObjectMeta{Name: "default", Namespace: "kube-system"}},
			},
			reactor: func(c *fake.Clientset) {
				c.PrependReactor("create", "secrets", func(action clienttesting.Action) (handled bool, ret runtime.Object, err error) {
					createAction := action.(clienttesting.CreateAction)
					obj := createAction.GetObject().(*corev1.Secret)
					if obj.Namespace == "kube-system" && obj.Name == SecretName {
						return true, nil, fmt.Errorf("fake kube-system secret create error")
					}
					return false, nil, nil
				})
			},
			wantErr: true, // Should fail because one namespace failed
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			fakeK8s := fake.NewSimpleClientset(tt.initObjects...)
			if tt.reactor != nil {
				tt.reactor(fakeK8s)
			}

			ctx := t.Context()

			err := UpdateGcrCredentials(ctx, fakeK8s, tt.tokenSource)
			if (err != nil) != tt.wantErr {
				t.Errorf("UpdateGcrCredentials() error = %v, wantErr %v", err, tt.wantErr)
				return
			}

			if tt.wantErr {
				return
			}

			// Verify secrets that should exist and be updated
			for _, ns := range tt.wantSecretNS {
				secret, err := fakeK8s.CoreV1().Secrets(ns).Get(ctx, SecretName, metav1.GetOptions{})
				if err != nil {
					t.Errorf("Expected secret %s in namespace %s, but got error: %v", SecretName, ns, err)
				} else {
					if len(secret.Data) == 0 {
						t.Errorf("Secret %s in namespace %s should have data", SecretName, ns)
					}
				}
			}

			// Verify secrets that should exist but NOT be updated (remain empty)
			for _, ns := range tt.wantUnchangedSecretNS {
				secret, err := fakeK8s.CoreV1().Secrets(ns).Get(ctx, SecretName, metav1.GetOptions{})
				if err != nil {
					t.Errorf("Expected secret %s to exist in namespace %s, but got error: %v", SecretName, ns, err)
				} else {
					if len(secret.Data) != 0 {
						t.Errorf("Secret %s in namespace %s should NOT have data (should be unchanged)", SecretName, ns)
					}
				}
			}

			// Verify secrets that should NOT exist
			for _, ns := range tt.dontWantSecretNS {
				_, err := fakeK8s.CoreV1().Secrets(ns).Get(ctx, SecretName, metav1.GetOptions{})
				if err == nil {
					t.Errorf("Expected secret %s to NOT exist in namespace %s, but it exists", SecretName, ns)
				} else if !k8serrors.IsNotFound(err) {
					t.Errorf("Expected NotFound error for secret %s in namespace %s, but got: %v", SecretName, ns, err)
				}
			}

			// Verify patched SAs
			for _, ns := range tt.wantPatchedNS {
				sa, err := fakeK8s.CoreV1().ServiceAccounts(ns).Get(ctx, "default", metav1.GetOptions{})
				if err != nil {
					t.Errorf("Expected SA default in namespace %s, but got error: %v", ns, err)
				} else {
					if len(sa.ImagePullSecrets) != 1 || sa.ImagePullSecrets[0].Name != SecretName {
						t.Errorf("SA default in namespace %s not patched correctly: %+v", ns, sa)
					}
				}
			}
		})
	}
}
