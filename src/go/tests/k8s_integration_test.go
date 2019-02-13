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
	"fmt"
	"log"
	"testing"
	"time"

	"github.com/googlecloudrobotics/core/src/go/pkg/kubeutils"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/client-go/kubernetes"
	_ "k8s.io/client-go/plugin/pkg/client/auth/gcp"
)

const (
	// In the K8s pod lifecycle, when a pod runs for ten minutes the
	// restart backoff is reset. As such, we can assume that a pod running
	// for >10 minutes is healthy.
	maxTimeSinceRelevantRestart = 10 * time.Minute

	podInitializationTimeoutSeconds  = 60 * 5
	podMaxToleratedContainerRestarts = 5
)

func checkHealthOfKubernetesCluster(kubernetesContext string) error {
	// create the kubernetes clientSet
	k8sCfg, err := kubeutils.LoadOutOfClusterConfig(kubernetesContext)
	if err != nil {
		return fmt.Errorf("Loading of kubernetes config failed: %v", err)
	}
	clientSet, err := kubernetes.NewForConfig(k8sCfg)
	if err != nil {
		return fmt.Errorf("Creating the kubernetes client set failed: %v", err)
	}

	numNonRunningPods := 0
	failingContainers := 0

	timeStart := time.Now()

	for time.Since(timeStart) < podInitializationTimeoutSeconds*time.Second {
		log.Printf("Querying pods from context %s...", kubernetesContext)
		pods, err := clientSet.CoreV1().Pods("").List(metav1.ListOptions{})
		if err != nil {
			return fmt.Errorf("Failed to query pods: %v", err)
		}
		log.Printf("...done. Found %d pods in the cluster.\n", len(pods.Items))

		if len(pods.Items) == 0 {
			return fmt.Errorf("Could not find any pods in cluster")
		}

		numNonRunningPods = 0
		failingContainers = 0
		for _, pod := range pods.Items {
			log.Printf("Pod %v is in state: %s\n", pod.Name, pod.Status.Phase)
			if pod.Status.Phase != "Running" && pod.Status.Phase != "Succeeded" {
				numNonRunningPods += 1
				break
			}

			waitingContainerFound := false
			for _, container := range pod.Status.ContainerStatuses {
				// Exactly one of Running/Terminated/Waiting in container.State is set
				if container.State.Running != nil {
					timeSinceLastRestart := time.Now().Sub(container.State.Running.StartedAt.Time)

					if container.RestartCount > podMaxToleratedContainerRestarts && timeSinceLastRestart < maxTimeSinceRelevantRestart {
						log.Printf("Warning: Container %s (%s) restarted %d times in pod %s\n", container.Name,
							container.Image, container.RestartCount, pod.Name)
						failingContainers += 1
					}
				} else if container.State.Terminated != nil && container.State.Terminated.ExitCode != 0 {
					log.Printf("Warning: Container %s (%s) was terminated in pod %s with error\n", container.Name, container.Image, pod.Name)
					failingContainers += 1
				} else if container.State.Waiting != nil {
					log.Printf("Warning: Container %s (%s) was waiting in pod %s with error\n", container.Name, container.Image, pod.Name)
					waitingContainerFound = true
				}
			}
			if waitingContainerFound {
				numNonRunningPods += 1
			}
		}

		if numNonRunningPods == 0 && failingContainers == 0 {
			break
		}

		time.Sleep(10 * time.Second)
	}

	if numNonRunningPods != 0 || failingContainers != 0 {
		return fmt.Errorf("Unhealthy cluster status after waiting for %d sec: %d non-running pods, %d failing containers\n",
			podInitializationTimeoutSeconds, numNonRunningPods, failingContainers)
	}
	log.Printf("All pods are happily running :)\n")
	return nil
}

func TestKubernetesCloudClusterStatus(t *testing.T) {
	kubernetesCloudContext, err := kubeutils.GetCloudKubernetesContext()
	if err != nil {
		t.Error(err)
	}

	if err := checkHealthOfKubernetesCluster(kubernetesCloudContext); err != nil {
		t.Errorf("Cloud cluster %s: %v", kubernetesCloudContext, err)
	}
}

func TestKubernetesRobotClusterStatus(t *testing.T) {
	kubernetesRobotContext, err := kubeutils.GetRobotKubernetesContext()
	if err != nil {
		t.Error(err)
	}

	if err := checkHealthOfKubernetesCluster(kubernetesRobotContext); err != nil {
		t.Errorf("Robot cluster %s: %v", kubernetesRobotContext, err)
	}

}
