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
	"bytes"
	"context"
	"errors"
	"flag"
	"fmt"
	"log"
	"net/http"
	"os"
	"os/exec"
	"regexp"
	"time"

	"github.com/cenkalti/backoff"
	"github.com/googlecloudrobotics/core/src/go/pkg/configutil"
	"github.com/googlecloudrobotics/core/src/go/pkg/kubeutils"
	"github.com/googlecloudrobotics/core/src/go/pkg/robotauth"
	"github.com/googlecloudrobotics/core/src/go/pkg/setup"
	"github.com/googlecloudrobotics/core/src/go/pkg/setup/util"

	"golang.org/x/oauth2"
	"golang.org/x/oauth2/google"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/client-go/dynamic"
	_ "k8s.io/client-go/plugin/pkg/client/auth/gcp"
	"k8s.io/client-go/tools/clientcmd"
	clientapi "k8s.io/client-go/tools/clientcmd/api"
)

const iotPrefix = "dev-"

var (
	project   = flag.String("project", "", "Project ID for the Google Cloud Platform")
	robotName = flag.String("robot-name", "", "Robot name (default: select interactively)")
)

func parseFlags() {
	flag.Parse()

	if *project == "" {
		fmt.Println("ERROR: --project not specified")
		os.Exit(1)
	}
}

func main() {
	parseFlags()
	ctx := context.Background()
	f := &util.DefaultFactory{}

	vars, err := configutil.ReadConfig(*project)
	if err != nil {
		log.Fatal("Failed to read config for project: ", err)
	}
	domain, ok := vars["CLOUD_ROBOTICS_DOMAIN"]
	if !ok || domain == "" {
		domain = fmt.Sprintf("www.endpoints.%s.cloud.goog", *project)
	}

	tokenSource, err := google.DefaultTokenSource(context.Background(), "https://www.googleapis.com/auth/cloud-platform")
	if err != nil {
		log.Fatal("Failed to create OAuth2 token source: ", err)
	}
	k8sCfg := kubeutils.BuildCloudKubernetesConfig(tokenSource, domain)
	k8s, err := dynamic.NewForConfig(k8sCfg)
	if err != nil {
		log.Fatal("Failed to create kubernetes client: ", err)
	}
	robotGVR := schema.GroupVersionResource{Group: "registry.cloudrobotics.com", Version: "v1alpha1", Resource: "robots"}
	robotClient := k8s.Resource(robotGVR).Namespace("default")

	*robotName, err = setup.GetRobotName(ctx, f, robotClient, *robotName)
	if err != nil {
		log.Fatalln("ERROR:", err)
	}
	if err := createKubeRelayEntry(*project, domain, *robotName); err != nil {
		log.Fatalln("Failed to create kubectl context:", err)
	}
	// TODO(ensonic): these are only used for the ssh-app
	// dev credentials are always created
	client := oauth2.NewClient(context.Background(), tokenSource)
	if err := setupDevCredentials(client, domain, *robotName); err != nil {
		log.Fatal(err)
	}
	log.Println("Setup complete.")
	return
}

// createKubeRelayEntry writes cluster configuration to ~/.kube/config,
// pointing to the kubernetes-relay-server running on GKE and linked to the
// given robot.
func createKubeRelayEntry(projectID string, domain string, robotName string) error {
	rules := clientcmd.NewDefaultClientConfigLoadingRules()
	config, err := rules.Load()
	if err != nil {
		return err
	}
	if config.AuthInfos["cloud-robotics-gcp"] == nil {
		config.AuthInfos["cloud-robotics-gcp"] = &clientapi.AuthInfo{
			AuthProvider: &clientapi.AuthProviderConfig{Name: "gcp"},
		}
	}
	name := fmt.Sprintf("%s-robot", projectID)
	url := fmt.Sprintf("https://%s/apis/core.kubernetes-relay/client/%s", domain, robotName)
	if config.Clusters[name] == nil {
		config.Clusters[name] = &clientapi.Cluster{}
	}
	// Always overwrite the URL because it encodes the robot we're using.
	config.Clusters[name].Server = url
	if config.Contexts[name] == nil {
		config.Contexts[name] = &clientapi.Context{
			AuthInfo:  "cloud-robotics-gcp",
			Cluster:   name,
			Namespace: "default",
		}
	}
	if err := clientcmd.WriteToFile(*config, rules.GetDefaultFilename()); err != nil {
		return err
	}
	fmt.Printf("Robot context created, use with:   kubectl --context %s\n", name)
	return nil
}

// setupDevCredentials generates a workstation ID for use with Cloud IoT then
// calls in to CreateAndPublishCredentialsToCloud to create and publish a private key.
func setupDevCredentials(client *http.Client, domain string, robotName string) error {
	hostname, err := os.Hostname()
	if err != nil {
		return fmt.Errorf("Failed to query hostname: %v", err)
	}
	auth := &robotauth.RobotAuth{
		RobotName:           robotName,
		ProjectId:           *project,
		Domain:              domain,
		PublicKeyRegistryId: makeIdentifier(hostname),
	}
	log.Println("Creating new private key")
	if err := auth.CreatePrivateKey(); err != nil {
		return err
	}
	if err := setup.PublishCredentialsToCloud(client, auth /*retries*/, 1); err != nil {
		return err
	}
	if err := auth.StoreInFile(); err != nil {
		return fmt.Errorf("Failed to store private key: %v", err)
	}
	return nil
}

// makeIdentifier converts a string to a valid Cloud IoT ID by adding a prefix
// and removing invalid characters.
func makeIdentifier(base string) string {
	invalid, err := regexp.Compile("[^a-zA-Z0-9_.~+%-]+")
	if err != nil {
		log.Fatalln("makeValidIdentifier: failed to compile regex:", err)
	}
	return iotPrefix + invalid.ReplaceAllString(base, "")
}

func containerExists(container string) (bool, error) {
	cmd := exec.Command("docker", "inspect", container)
	output, err := cmd.CombinedOutput()
	if err != nil && bytes.HasPrefix(output, []byte("Error: No such object")) {
		return false, nil
	} else if err != nil {
		return false, fmt.Errorf("`docker inspect %s` failed (%v): %s", container, err, output)
	}
	return true, nil
}

// stopContainerIfNeeded stops a container if it exists, then waits for it to
// be automatically deleted. It assumes the container was run with --rm.
func stopContainerIfNeeded(container string) error {
	if exists, err := containerExists(container); err != nil {
		return err
	} else if !exists {
		return nil
	}
	log.Printf("Stopping %s container", container)
	cmd := exec.Command("docker", "stop", container)
	cmd.Stderr = os.Stderr
	if err := cmd.Run(); err != nil {
		return fmt.Errorf("`docker stop %s` failed: %v", container, err)
	}
	// Wait for the container to be deleted.
	return backoff.Retry(
		func() error {
			if stillExists, err := containerExists(container); err != nil {
				return backoff.Permanent(err)
			} else if stillExists {
				return errors.New("container exists")
			}
			return nil
		},
		backoff.NewConstantBackOff(100*time.Millisecond),
	)
}
