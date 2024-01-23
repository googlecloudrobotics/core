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
	"log/slog"
	"testing"
	"time"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"github.com/googlecloudrobotics/core/src/go/pkg/kubeutils"
	"github.com/googlecloudrobotics/ilog"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/client-go/kubernetes"
	"k8s.io/client-go/kubernetes/scheme"
	ctrlclient "sigs.k8s.io/controller-runtime/pkg/client"
)

const (
	appInitializationTimeout = 7 * time.Minute
	podInitializationTimeout = 5 * time.Minute
)

func checkHealthOfKubernetesCluster(ctx context.Context, kubernetesContext string) error {
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

	restartCount := make(map[string]int32)

	timeStart := time.Now()

	for time.Since(timeStart) < podInitializationTimeout {
		slog.Info("Querying pods...", slog.String("Context", kubernetesContext))
		pods, err := clientSet.CoreV1().Pods("").List(ctx, metav1.ListOptions{})
		if err != nil {
			return fmt.Errorf("Failed to query pods: %v", err)
		}
		slog.Info("...done.", slog.Int("PodCount", len(pods.Items)))

		if len(pods.Items) == 0 {
			return fmt.Errorf("Could not find any pods in cluster")
		}

		numNonRunningPods = 0
		failingContainers = 0
		for _, pod := range pods.Items {
			slog.Info("Pod state", slog.String("Name", pod.Name), slog.String("Phase", string(pod.Status.Phase)))
			if pod.Status.Phase != "Running" && pod.Status.Phase != "Succeeded" {
				numNonRunningPods += 1
				break
			}

			waitingContainerFound := false
			for _, container := range pod.Status.ContainerStatuses {
				// Exactly one of Running/Terminated/Waiting in container.State is set
				if container.State.Running != nil {
					restartKey := pod.Name + container.Name
					prevRestarts, ok := restartCount[restartKey]
					if !ok {
						prevRestarts = 0
					}
					if container.RestartCount > prevRestarts {
						slog.Warn("Container restarted",
							slog.String("Pod", pod.Name),
							slog.String("Container", container.Name),
							slog.String("Image", container.Image),
							slog.Int("RestartCount", int(container.RestartCount)))
						failingContainers += 1
					}
					restartCount[restartKey] = container.RestartCount
				} else if container.State.Terminated != nil && container.State.Terminated.ExitCode != 0 {
					slog.Warn("Container terminated",
						slog.String("Pod", pod.Name),
						slog.String("Container", container.Name),
						slog.String("Image", container.Image),
						slog.Int("RestartCount", int(container.RestartCount)))
					failingContainers += 1
				} else if container.State.Waiting != nil {
					slog.Warn("Container waiting",
						slog.String("Pod", pod.Name),
						slog.String("Container", container.Name),
						slog.String("Image", container.Image),
						slog.Int("RestartCount", int(container.RestartCount)))
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
			podInitializationTimeout/time.Second, numNonRunningPods, failingContainers)
	}
	slog.Info("All pods are happily running :)")
	return nil
}

// convert a resource from one type representation to another one.
func convert(from, to runtime.Object) error {
	b, err := json.Marshal(from)
	if err != nil {
		return err
	}
	return json.Unmarshal(b, &to)
}

func TestCloudClusterAppStatus(t *testing.T) {
	kubernetesContext, err := kubeutils.GetCloudKubernetesContext()
	if err != nil {
		t.Error(err)
	}
	k8sCfg, err := kubeutils.LoadOutOfClusterConfig(kubernetesContext)
	if err != nil {
		t.Errorf("Loading of kubernetes config failed: %v", err)
	}

	sc := runtime.NewScheme()
	scheme.AddToScheme(sc)
	apps.AddToScheme(sc)

	client, err := ctrlclient.New(k8sCfg, ctrlclient.Options{Scheme: sc})
	if err != nil {
		t.Errorf("Failed to create kubernetes client: %v", err)
	}

	numBadConditions := 0

	timeStart := time.Now()

	for time.Since(timeStart) < appInitializationTimeout {
		appRollouts := &unstructured.UnstructuredList{}
		appRollouts.SetGroupVersionKind(schema.GroupVersionKind{
			Group:   "apps.cloudrobotics.com",
			Kind:    "AppRollout",
			Version: "v1alpha1",
		})
		slog.Info("Querying AppRollouts...", slog.String("Context", kubernetesContext))
		err = client.List(context.Background(), appRollouts)
		if err != nil {
			slog.Error("Failed to list AppRollouts", ilog.Err(err))
			time.Sleep(10 * time.Second)
			continue
		}
		slog.Info("...done.", slog.Int("AppRolloutCount", len(appRollouts.Items)))

		numBadConditions = 0
		for _, i := range appRollouts.Items {
			ar := &apps.AppRollout{}
			if err := convert(&i, ar); err != nil {
				t.Errorf("Failed to unmarshall AppRollout: %v", err)
			}
			for _, c := range ar.Status.Conditions {
				slog.Info("AppRollout condition", slog.String("Name", ar.GetName()), slog.String("Condition", string(c.Type)), slog.String("Status", string(c.Status)))
				if c.Status != corev1.ConditionTrue {
					slog.Warn("AppRollout condition not met", slog.String("Name", ar.GetName()), slog.String("Condition", string(c.Type)))
					numBadConditions += 1
				}
			}
		}
		if numBadConditions == 0 {
			break
		}

		time.Sleep(15 * time.Second)
	}
	if numBadConditions != 0 {
		t.Errorf("Unhealthy AppRollout status after waiting for %d sec: %d conditions not met\n",
			appInitializationTimeout/time.Second, numBadConditions)
	}
}

func TestKubernetesCloudClusterStatus(t *testing.T) {
	ctx := context.Background()
	kubernetesCloudContext, err := kubeutils.GetCloudKubernetesContext()
	if err != nil {
		t.Error(err)
	}

	if err := checkHealthOfKubernetesCluster(ctx, kubernetesCloudContext); err != nil {
		t.Errorf("Cloud cluster %s: %v", kubernetesCloudContext, err)
	}
}

func TestKubernetesRobotClusterStatus(t *testing.T) {
	ctx := context.Background()
	kubernetesRobotContext, err := kubeutils.GetRobotKubernetesContext()
	if err != nil {
		t.Error(err)
	}

	if err := checkHealthOfKubernetesCluster(ctx, kubernetesRobotContext); err != nil {
		t.Errorf("Robot cluster %s: %v", kubernetesRobotContext, err)
	}

}
