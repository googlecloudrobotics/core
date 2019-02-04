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

package main

import (
	"fmt"
	"log"
	"testing"
	"time"

	"cloud-robotics.googlesource.com/cloud-robotics/pkg/utils"
	"cloud.google.com/go/pubsub"
	"golang.org/x/net/context"
	"google.golang.org/api/iterator"

	_ "k8s.io/client-go/plugin/pkg/client/auth/gcp"

	"cloud-robotics.googlesource.com/cloud-robotics/pkg/kubeutils"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/client-go/dynamic"
)

const (
	waitTimeout = 120 * time.Second
)

func getRobotClient() (dynamic.ResourceInterface, error) {
	kubernetesContext, err := kubeutils.GetCloudKubernetesContext()
	if err != nil {
		return nil, fmt.Errorf("failed to get cloud context: %v", err)
	}
	kubernetesConfig, err := kubeutils.LoadOutOfClusterConfig(kubernetesContext)
	if err != nil {
		return nil, fmt.Errorf("failed to initialize Kubernetes config: %v", err)
	}
	k8sClient, err := dynamic.NewForConfig(kubernetesConfig)
	if err != nil {
		return nil, fmt.Errorf("failed to initialize Kubernetes client: %v", err)
	}

	robotGVR := schema.GroupVersionResource{Group: "registry.cloudrobotics.com", Version: "v1alpha1", Resource: "robots"}
	robotClient := k8sClient.Resource(robotGVR).Namespace("default")

	return robotClient, nil
}

func createTestRobot(robotClient dynamic.ResourceInterface) (*unstructured.Unstructured, error) {
	robot := &unstructured.Unstructured{
		Object: map[string]interface{}{
			"apiVersion": "registry.cloudrobotics.com/v1alpha1",
			"kind":       "Robot",
			"metadata": map[string]interface{}{
				"namespace": "default",
				"name":      "go-test-robot",
				"labels": map[string]interface{}{
					"cloudrobotics.com/robot-name": "go-test-robot",
				},
			},
			// TODO(ensonic): should we put r.Name into a displayName field?
			"spec": map[string]interface{}{
				"type": "Mir100",
				"role": "RoboLab",
			},
		},
	}

	obj, err := robotClient.Create(robot, metav1.CreateOptions{})
	if err != nil {
		return nil, fmt.Errorf("failed to create robot CR: %v", err)
	}
	return obj, nil
}

func deleteTestRobot(robotClient dynamic.ResourceInterface, robot *unstructured.Unstructured) error {
	return robotClient.Delete(robot.GetName(), &metav1.DeleteOptions{})
}

func waitForPubSubTopicChange(topicSearch string, waitForCreation bool) error {
	ctx := context.Background()
	pubsubClient, err := pubsub.NewClient(ctx, utils.MustGetenv("GCP_PROJECT_ID"))
	if err != nil {
		return fmt.Errorf("Could not create pubsub client: %v", err)
	}

	timeStart := time.Now()

	for time.Since(timeStart) < waitTimeout {
		if waitForCreation {
			fmt.Printf("Waiting for the creation of pubsub topic: %s...\n", topicSearch)
		} else {
			fmt.Printf("Waiting for the deletion of pubsub topic: %s...\n", topicSearch)
		}
		it := pubsubClient.Topics(ctx)
		topicFound := false
		for {
			topic, err := it.Next()
			if err == iterator.Done {
				break
			}
			if err != nil {
				return err
			}
			if topic.ID() == topicSearch {
				topicFound = true
			}
		}
		if topicFound && waitForCreation {
			return nil
		}
		if !topicFound && !waitForCreation {
			return nil
		}
		time.Sleep(5 * time.Second)
	}
	return fmt.Errorf("Timeout occurred")
}

// TestRobotIntegration uses the Registry API to create, modify and delete a
// robot while expecting Helm to bring up and shutdown corresponding pods and
// the cloud-master to create and delete the deployment status pub-sub topic.
func TestRobotIntegration(t *testing.T) {
	robotClient, err := getRobotClient()
	if err != nil {
		t.Errorf("Failed to get robotClient: %v", err)
	}

	log.Println("Create test robot")
	robot, err := createTestRobot(robotClient)
	if err != nil {
		t.Errorf("Failed to register test robot: %v", err)
	}
	log.Printf("Robot created with Name: %s\n", robot.GetName())

	deploymentTopic := fmt.Sprintf("robots.%s.deployment", robot.GetName())
	err = waitForPubSubTopicChange(deploymentTopic /*waitForCreation=*/, true)
	if err != nil {
		t.Errorf("Failed to detect pubsub topic %s: %v", deploymentTopic, err)
	}

	log.Printf("Deleting robot\n")
	err = deleteTestRobot(robotClient, robot)
	if err != nil {
		t.Errorf("Deleting robot failed: %v", err)
	}

	err = waitForPubSubTopicChange(deploymentTopic /*waitForCreation=*/, false)
	if err != nil {
		t.Errorf("Failed to remove pubsub topic %s: %v", deploymentTopic, err)
	}
}
