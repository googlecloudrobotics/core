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
	"context"
	"crypto/rsa"
	"crypto/x509"
	"encoding/pem"
	"fmt"
	"log"
	"net"
	"net/http"
	"os"
	"strings"
	"time"

	"github.com/cenkalti/backoff"
	"github.com/googlecloudrobotics/core/src/go/pkg/robotauth"
	"github.com/googlecloudrobotics/core/src/go/pkg/setup/util"

	"golang.org/x/crypto/ssh/terminal"
	apierrors "k8s.io/apimachinery/pkg/api/errors"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"k8s.io/client-go/dynamic"
	_ "k8s.io/client-go/plugin/pkg/client/auth/gcp"
)

// GetRobotName returns a valid robot name or an error. If the robotName parameter
// is non-empty, it checks if it is valid. If it is an empty string, the user is
// prompted to select a robot.
func GetRobotName(ctx context.Context, f util.Factory, client dynamic.ResourceInterface, robotName string) (string, error) {
	if robotName == "" {
		exitIfNotRunningInTerminal("ERROR: --robot-name not specified")

		robots, err := client.List(ctx, metav1.ListOptions{})
		if err != nil {
			return "", err
		}

		robotName, err := selectRobot(f, robots.Items)
		if err != nil {
			return "", err
		}
		return robotName, nil
	}
	_, err := client.Get(ctx, robotName, metav1.GetOptions{})
	if err != nil {
		if s, ok := err.(*apierrors.StatusError); ok && s.ErrStatus.Reason == metav1.StatusReasonNotFound {
			return "", fmt.Errorf("Robot %v not found.", robotName)
		}
		return "", err
	}
	return robotName, nil
}

// exitIfNotRunningInTerminal checks if stdin is connected to a terminal. If
// not, it prints the given message and exits.
func exitIfNotRunningInTerminal(message ...interface{}) {
	if !terminal.IsTerminal(int(os.Stdin.Fd())) {
		fmt.Fprintln(os.Stderr, message...)
		os.Exit(1)
	}
}

// Ask the user to select the robot from a list. Saves name to disk after
// selection.
func selectRobot(f util.Factory, robots []unstructured.Unstructured) (string, error) {
	fmt.Printf("  # %-20v %-10v %-16v\n", "Name", "Type", "Create Time")
	for i, robot := range robots {
		spec, ok := robot.Object["spec"].(map[string]interface{})
		if !ok {
			log.Print("unmarshaling robot failed: spec is not a map")
			continue
		}
		fmt.Printf("%3v %-20v %-10v %v\n", i+1, robot.GetName(), spec["type"], robot.GetCreationTimestamp().String())
	}

	fmt.Print("Select robot: ")
	var ix int
	for {
		var err error
		ix, err = f.ScanInt()
		if err == nil && 1 <= ix && ix <= len(robots) {
			break
		}
		fmt.Printf("Please enter a number (1-%v): ", len(robots))
	}
	return robots[ix-1].GetName(), nil
}

func newExponentialBackoff(initialInterval time.Duration, multiplier float64, retries uint64) backoff.BackOff {
	exponentialBackoff := backoff.ExponentialBackOff{
		InitialInterval: initialInterval,
		Multiplier:      multiplier,
		Clock:           backoff.SystemClock,
	}
	exponentialBackoff.Reset()
	return backoff.WithMaxRetries(&exponentialBackoff, retries)
}

// WaitForDNS manually resolves the domain name with retries to give a better
// error in the case of failure. This is useful to catch errors during first
// interaction with the cluster and cloud-project,
func WaitForDNS(domain string, retries uint64) error {
	log.Printf("DNS lookup for %q", domain)

	if err := backoff.RetryNotify(
		func() error {
			ips, err := net.LookupIP(domain)
			if err != nil {
				return err
			}

			// Check that the results contain an ipv4 addr. Initially, coredns may only
			// return ipv6 addresses in which case helm will fail.
			for _, ip := range ips {
				if ip.To4() != nil {
					return nil
				}
			}

			return fmt.Errorf("IP not found")
		},
		newExponentialBackoff(time.Second, 2, retries),
		func(_ error, _ time.Duration) {
			log.Printf("... Retry dns for %q", domain)
		},
	); err != nil {
		return fmt.Errorf("DNS lookup for %q failed: %w", domain, err)
	}

	return nil
}

// WaitForService tests a given cloud endpoint with a HEAD request a few times.
// This lets us wait for the service to be available or error with a better
// message.
func WaitForService(client *http.Client, url string, retries uint64) error {
	log.Printf("Service probe for %q", url)

	if err := backoff.RetryNotify(
		func() error {
			_, err := client.Head(url)
			return err
		},
		newExponentialBackoff(time.Second, 2, retries),
		func(_ error, _ time.Duration) {
			log.Printf("... Retry service for %q", url)
		},
	); err != nil {
		return fmt.Errorf("service probe for %q failed: %w", url, err)
	}

	return nil
}

// PublishCredentialsToCloud registers a public-key in the cloud under the ID
// given as part of the RobotAuth struct.
func PublishCredentialsToCloud(client *http.Client, auth *robotauth.RobotAuth, retries uint64) error {
	if len(auth.PrivateKey) == 0 {
		return fmt.Errorf("Missing key in given auth object")
	}
	if err := isKeyRegistryAvailable(auth, client, retries); err != nil {
		return fmt.Errorf("Failed to connect to cloud key registry: %w", err)
	}
	if err := publishPublicKeyToCloudRegistry(auth, client); err != nil {
		return fmt.Errorf("Failed to register key with cloud key registry: %w", err)
	}
	return nil
}

func isKeyRegistryAvailable(auth *robotauth.RobotAuth, client *http.Client, retries uint64) error {
	// Make sure the cloud cluster take requests
	url := fmt.Sprintf("https://%s/apis/core.token-vendor/v1/public-key.read", auth.Domain)
	if err := WaitForService(client, url, retries); err != nil {
		return fmt.Errorf("Failed to connect to the cloud cluster: %w. Please retry in 5 minutes.", err)
	}
	return nil
}

func publishPublicKeyToCloudRegistry(auth *robotauth.RobotAuth, client *http.Client) error {
	pubKey, err := getPublicKey(auth.PrivateKey)
	if err != nil {
		return err
	}

	log.Println("Publishing the robot's public key to cloud key registry")

	url := fmt.Sprintf(
		"https://%s/apis/core.token-vendor/v1/public-key.publish?device-id=%s",
		auth.Domain,
		auth.PublicKeyRegistryId)

	response, err := client.Post(
		url, "application/x-pem-file", strings.NewReader(string(pubKey)))

	if err != nil {
		return fmt.Errorf("publishing the token failed: %w", err)
	} else if response.StatusCode != http.StatusOK {
		responseBody := new(bytes.Buffer)
		responseBody.ReadFrom(response.Body)
		return fmt.Errorf(
			"TokenVendor responded with %d %s: %s",
			response.StatusCode,
			response.Status,
			responseBody.String())
	}

	return nil
}

func getPublicKey(privateKey []byte) ([]byte, error) {
	block, _ := pem.Decode(privateKey)
	if block == nil {
		return nil, fmt.Errorf("Private key is not a valid PEM object")
	}
	var rsaKey *rsa.PrivateKey
	if block.Type == "RSA PRIVATE KEY" {
		key, err := x509.ParsePKCS1PrivateKey(block.Bytes)
		if err != nil {
			return nil, err
		}
		rsaKey = key
	} else if block.Type == "PRIVATE KEY" {
		key, err := x509.ParsePKCS8PrivateKey(block.Bytes)
		if err != nil {
			return nil, err
		}
		rsaKey = key.(*rsa.PrivateKey)
	} else {
		return nil, fmt.Errorf("Expected a private key, got %s", block.Type)
	}
	pubKey, err := x509.MarshalPKIXPublicKey(&rsaKey.PublicKey)
	if err != nil {
		return nil, err
	}
	return pem.EncodeToMemory(&pem.Block{
		Type:  "PUBLIC KEY",
		Bytes: pubKey,
	}), nil
}

// megeMaps returns `base` with `additions` added on top.
// I.e., if the same key is present in both maps, the one from `additions` wins.
func mergeMaps(base, additions map[string]string) map[string]string {
	result := make(map[string]string)
	for k, v := range base {
		result[k] = v
	}
	for k, v := range additions {
		result[k] = v
	}
	return result
}

// CreateOrUpdateRobot adds a new robot-cr or updates an existing one.
func CreateOrUpdateRobot(ctx context.Context, client dynamic.ResourceInterface, robotName, robotType, project string, labels map[string]string, annotations map[string]string) error {
	robot, err := client.Get(ctx, robotName, metav1.GetOptions{})
	if err != nil {
		if s, ok := err.(*apierrors.StatusError); ok && s.ErrStatus.Reason == metav1.StatusReasonNotFound {
			robot := &unstructured.Unstructured{}
			robot.SetKind("Robot")
			robot.SetAPIVersion("registry.cloudrobotics.com/v1alpha1")
			robot.SetName(robotName)

			robot.SetLabels(labels)
			robot.SetAnnotations(annotations)
			robot.Object["spec"] = map[string]interface{}{
				"type":    robotType,
				"project": project,
			}
			robot.Object["status"] = make(map[string]interface{})
			_, err := client.Create(ctx, robot, metav1.CreateOptions{})
			return err
		} else {
			return fmt.Errorf("Failed to get robot %v: %w", robotName, err)
		}
	}

	// A robot with the same name already exists.
	robot.SetLabels(mergeMaps(robot.GetLabels(), labels))
	robot.SetAnnotations(mergeMaps(robot.GetAnnotations(), annotations))
	spec, ok := robot.Object["spec"].(map[string]interface{})
	if !ok {
		return fmt.Errorf("unmarshaling robot failed: spec is not a map")
	}
	spec["type"] = robotType
	spec["project"] = project
	_, err = client.Update(ctx, robot, metav1.UpdateOptions{})
	return err
}
