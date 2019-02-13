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
	"context"
	"encoding/json"
	"fmt"
	"log"
	"strings"

	"cloud.google.com/go/pubsub"
	"google.golang.org/api/iterator"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/client-go/dynamic"
	"k8s.io/client-go/rest"
)

type statusPublisher struct {
	robotClient dynamic.ResourceInterface
	psClient    *pubsub.Client
	robotTopics map[string]*pubsub.Topic
}

func newStatusPublisher(cloudProjectName string, kubernetesConfig *rest.Config) (*statusPublisher, error) {
	k8sClient, err := dynamic.NewForConfig(kubernetesConfig)
	if err != nil {
		return nil, err
	}
	robotGVR := schema.GroupVersionResource{Group: "registry.cloudrobotics.com", Version: "v1alpha1", Resource: "robots"}
	robotClient := k8sClient.Resource(robotGVR).Namespace("default")

	psClient, err := pubsub.NewClient(context.Background(), cloudProjectName)
	if err != nil {
		return nil, err
	}
	robotTopics := make(map[string]*pubsub.Topic)
	return &statusPublisher{robotClient, psClient, robotTopics}, nil
}

func (sp *statusPublisher) spin() error {
	idSet, err := sp.listRobotsInCluster()
	if err != nil {
		return err
	}

	for id := range idSet {
		sp.publishStatus(id)
	}

	sp.deleteStaleTopics(idSet)
	if err != nil {
		log.Printf("stale topic deletion failed: %v", err)
	}
	sp.deleteStaleSubscriptions()

	return nil
}

func (sp *statusPublisher) publishStatus(robotName string) {
	_, err := sp.robotClient.Get(robotName, metav1.GetOptions{})
	if err != nil {
		log.Printf("failed to get Robot '%s': %v", robotName, err)
		return
	}

	// TODO(ensonic): relay robot status details
	// status := robot.Object["status"]
	log.Printf("Robot %s exists", robotName)
	sp.notifyStatus(robotName, "ready")
}

func (sp *statusPublisher) notifyStatus(robotName string, status string) {
	topic, ok := sp.robotTopics[robotName]
	if !ok {
		topicID := fmt.Sprintf("robots.%s.deployment", robotName)
		topic = sp.psClient.Topic(topicID)
		topicExists, err := topic.Exists(context.Background())
		if !topicExists || err != nil {
			if err != nil {
				log.Printf("Checking topic for %s failed, status='%s'", robotName, err)
			}
			topic, err = sp.psClient.CreateTopic(context.Background(), topicID)
			if err != nil {
				log.Printf("Creating topic for %s failed, status='%s'", robotName, err)
				return
			}
			log.Printf("Created topic for %s", robotName)
		}
		sp.robotTopics[robotName] = topic
	}
	jsonData, err := json.Marshal(map[string]string{"status": status})
	if err != nil {
		log.Printf("Marshalling '%s' for topic for %d failed, status='%s'", status, robotName, err)
		return
	}
	log.Printf("notify: %s : %s", topic.ID(), jsonData)
	topic.Publish(context.Background(), &pubsub.Message{Data: jsonData})
}

func (sp *statusPublisher) deleteStaleTopics(idSet map[string]bool) {
	ctx := context.Background()
	topicIt := sp.psClient.Topics(ctx)
	for {
		topic, err := topicIt.Next()
		if err == iterator.Done {
			break
		}
		if err != nil {
			log.Printf("Error listing pubsub topics, status='%s'", err)
			break
		}
		parts := strings.SplitN(topic.ID(), ".", 3)
		if len(parts) == 3 && parts[0] == "robots" {
			robotName := parts[1]
			if !idSet[robotName] {
				sp.notifyStatus(robotName, "deleted")
				err := topic.Delete(ctx)
				if err != nil {
					log.Printf("Error deleting pubsub topic '%s', status='%s'", topic.ID(), err)
					continue
				}
				delete(sp.robotTopics, robotName)
				log.Printf("Deleted stale topic for %s", robotName)
			}
		}
	}
}

func (sp *statusPublisher) deleteStaleSubscriptions() {
	ctx := context.Background()
	subscriptionIt := sp.psClient.Subscriptions(ctx)
	for {
		subscription, err := subscriptionIt.Next()
		if err == iterator.Done {
			break
		}
		if err != nil {
			log.Printf("Error listing pubsub subscriptions, status='%s'", err)
			break
		}
		config, err := subscription.Config(ctx)
		if err != nil {
			log.Printf("Error getting config for '%s', status='%s'", subscription.ID(), err)
			continue
		}
		topicExists, err := config.Topic.Exists(ctx)
		if err != nil {
			log.Printf("Error checking topic for '%s', status='%s'", subscription.ID(), err)
			continue
		}
		if !topicExists {
			err := subscription.Delete(ctx)
			if err != nil {
				log.Printf("Failed to remove subscription '%s', status='%s'", subscription.ID(), err)
			}
		}
	}
}

func (sp *statusPublisher) listRobotsInCluster() (map[string]bool, error) {
	listResponse, err := sp.robotClient.List(metav1.ListOptions{})
	if err != nil {
		return nil, fmt.Errorf("failed to list robots: %v", err)
	}

	idSet := make(map[string]bool)
	for _, robot := range listResponse.Items {
		idSet[robot.GetName()] = true
	}

	return idSet, nil
}
