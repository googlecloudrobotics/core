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
	"fmt"
	"log"
	"net/http"
	"os"
	"strings"

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
func GetRobotName(f util.Factory, client dynamic.ResourceInterface, robotName string) (string, error) {
	if robotName == "" {
		exitIfNotRunningInTerminal("ERROR: --robot-name not specified")

		robots, err := client.List(metav1.ListOptions{})
		if err != nil {
			return "", err
		}

		robotName, err := selectRobot(f, robots.Items)
		if err != nil {
			return "", err
		}
		return robotName, nil
	}
	_, err := client.Get(robotName, metav1.GetOptions{})
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
	fmt.Printf("  # %-20v %-10v %-16v %v\n", "Name", "Type", "Create Time")
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

// Creates a private key and registers it in the cloud under the ID given
// as part of the RobotAuth struct. The private key is written to the
// RobotAuth struct.
func CreateAndPublishCredentialsToCloud(client *http.Client, auth *robotauth.RobotAuth) error {
	if err := createPrivateKey(auth); err != nil {
		return fmt.Errorf("Failed to create private key: %v", err)
	}
	if err := publishPublicKeyToCloudRegistry(auth, client); err != nil {
		return fmt.Errorf("Failed to register key with Cloud IoT: %v", err)
	}
	return nil
}

func createPrivateKey(auth *robotauth.RobotAuth) error {
	log.Println("Creating new private key")
	key, err := rsa.GenerateKey(rand.Reader, 2048)
	if err != nil {
		return err
	}
	pkcs8, err := x509.MarshalPKCS8PrivateKey(key)
	if err != nil {
		return err
	}
	auth.PrivateKey = pem.EncodeToMemory(&pem.Block{
		Type:  "PRIVATE KEY",
		Bytes: pkcs8,
	})
	return nil
}

func publishPublicKeyToCloudRegistry(auth *robotauth.RobotAuth, client *http.Client) error {
	pubKey, err := getPublicKey(auth.PrivateKey)
	if err != nil {
		return err
	}

	log.Println("Publishing the robot's public key to Cloud IoT Registry")

	url := fmt.Sprintf(
		"https://%s/apis/core.token-vendor/v1/public-key.publish?device-id=%s",
		auth.Domain,
		auth.PublicKeyRegistryId)

	response, err := client.Post(
		url, "application/x-pem-file", strings.NewReader(string(pubKey)))

	if err != nil {
		return fmt.Errorf("publishing the token failed: %v", err)
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
