// Copyright 2019 The Google Cloud Robotics Authors
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

// This file keeps the OAuth token in the alertmanager's config file updated.
package main

import (
	"context"
	"fmt"
	"log"
	"time"

	"github.com/ghodss/yaml"
	"golang.org/x/oauth2/google"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	typedv1 "k8s.io/client-go/kubernetes/typed/core/v1"
)

// writeAlertmanagerToken tries to update the token in the Kubernetes secret.
func writeAlertmanagerToken(corev1client typedv1.CoreV1Interface, token string) error {
	s, err := corev1client.Secrets(*targetNamespace).Get("alertmanager-kube-prometheus", metav1.GetOptions{})
	if err != nil {
		return fmt.Errorf("Failed to read secret: %v", err)
	}
	config := make(map[string]interface{})
	if err := yaml.Unmarshal(s.Data["alertmanager.yaml"], &config); err != nil {
		return fmt.Errorf("Unable to unmarshal alertmanager.yaml: %v", err)
	}
	if config["global"] == nil {
		config["global"] = make(map[string]interface{})
	}
	global, ok := config["global"].(map[string]interface{})
	if !ok {
		return fmt.Errorf("Expected global to be a dict")
	}
	if global["http_config"] == nil {
		global["http_config"] = make(map[string]interface{})
	}
	httpConfig, ok := global["http_config"].(map[string]interface{})
	if !ok {
		return fmt.Errorf("Expected global.http_config to be a dict")
	}
	httpConfig["bearer_token"] = token
	b, err := yaml.Marshal(config)
	if err != nil {
		return fmt.Errorf("Unable to remarshal alertmanager.yaml: %v", err)
	}
	s.Data["alertmanager.yaml"] = b
	if _, err := corev1client.Secrets(*targetNamespace).Update(s); err != nil {
		return fmt.Errorf("Failed to write secret: %v", err)
	}
	return nil
}

// keepAlertmanagerTokenUpdated loops forever updating the alertmanager token.
func keepAlertmanagerTokenUpdated(corev1client typedv1.CoreV1Interface) {
	ctx := context.Background()
	tokenSource, err := google.DefaultTokenSource(ctx, "email")
	if err != nil {
		log.Fatalf("Failed to build OAuth2 token source: %v", err)
	}
	for {
		token, err := tokenSource.Token()
		if err != nil {
			log.Printf("Failed to obtain oauth token: %v", err)
			time.Sleep(10 * time.Second)
			continue
		}
		if err := writeAlertmanagerToken(corev1client, token.AccessToken); err != nil {
			log.Printf("Failed to store oauth token: %v", err)
			time.Sleep(10 * time.Second)
			continue
		}
		refreshTime := token.Expiry.Add(-5 * time.Minute)
		sleepInterval := refreshTime.Sub(time.Now())
		log.Printf("Refreshed access token in alertmanager secret, sleeping for %f seconds", sleepInterval.Seconds())
		time.Sleep(sleepInterval)
	}
}
