// Copyright 2026 The Cloud Robotics Authors
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
	"time"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"github.com/googlecloudrobotics/core/src/go/pkg/kubeutils"
	"github.com/googlecloudrobotics/ilog"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/apimachinery/pkg/util/wait"
	"k8s.io/client-go/kubernetes"
	"k8s.io/client-go/kubernetes/scheme"
	"k8s.io/client-go/rest"
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
		return fmt.Errorf("loading of kubernetes config failed: %v", err)
	}
	return checkClusterHealth(ctx, kubernetesContext, k8sCfg)
}

func checkClusterHealth(ctx context.Context, kubernetesContext string, k8sCfg *rest.Config) error {
	clientSet, err := kubernetes.NewForConfig(k8sCfg)
	if err != nil {
		return fmt.Errorf("creating the kubernetes client set failed: %v", err)
	}

	numNonRunningPods := 0
	failingContainers := 0

	restartCount := make(map[string]int32)

	err = wait.PollUntilContextTimeout(ctx, 10*time.Second, podInitializationTimeout, true, func(ctx context.Context) (bool, error) {
		slog.InfoContext(ctx, "Querying pods...", slog.String("Context", kubernetesContext))
		pods, err := clientSet.CoreV1().Pods("").List(ctx, metav1.ListOptions{})
		if err != nil {
			return false, fmt.Errorf("failed to query pods: %v", err)
		}
		slog.InfoContext(ctx, "...done.", slog.Int("PodCount", len(pods.Items)))

		if len(pods.Items) == 0 {
			return false, fmt.Errorf("could not find any pods in cluster")
		}

		numNonRunningPods = 0
		failingContainers = 0
		for _, pod := range pods.Items {
			slog.InfoContext(ctx, "Pod state", slog.String("Name", pod.Name), slog.String("Phase", string(pod.Status.Phase)))
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
						slog.WarnContext(ctx, "Container restarted",
							slog.String("Pod", pod.Name),
							slog.String("Container", container.Name),
							slog.String("Image", container.Image),
							slog.Int("RestartCount", int(container.RestartCount)))
						failingContainers += 1
					}
					restartCount[restartKey] = container.RestartCount
				} else if container.State.Terminated != nil && container.State.Terminated.ExitCode != 0 {
					slog.WarnContext(ctx, "Container terminated",
						slog.String("Pod", pod.Name),
						slog.String("Container", container.Name),
						slog.String("Image", container.Image),
						slog.Int("RestartCount", int(container.RestartCount)))
					failingContainers += 1
				} else if container.State.Waiting != nil {
					slog.WarnContext(ctx, "Container waiting",
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

		return numNonRunningPods == 0 && failingContainers == 0, nil
	})
	if err != nil {
		if numNonRunningPods != 0 || failingContainers != 0 {
			return fmt.Errorf("unhealthy cluster status after waiting for %d sec: %d non-running pods, %d failing containers",
				podInitializationTimeout/time.Second, numNonRunningPods, failingContainers)
		}
		return err
	}

	slog.InfoContext(ctx, "All pods are happily running :)")
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

func checkAppRollouts(ctx context.Context, kubernetesContext string, k8sCfg *rest.Config) error {
	sc := runtime.NewScheme()
	scheme.AddToScheme(sc)
	apps.AddToScheme(sc)

	client, err := ctrlclient.New(k8sCfg, ctrlclient.Options{Scheme: sc})
	if err != nil {
		return fmt.Errorf("failed to create kubernetes client: %v", err)
	}

	numBadConditions := 0

	err = wait.PollUntilContextTimeout(ctx, 15*time.Second, appInitializationTimeout, true, func(ctx context.Context) (bool, error) {
		appRollouts := &unstructured.UnstructuredList{}
		appRollouts.SetGroupVersionKind(schema.GroupVersionKind{
			Group:   "apps.cloudrobotics.com",
			Kind:    "AppRollout",
			Version: "v1alpha1",
		})
		slog.InfoContext(ctx, "Querying AppRollouts...", slog.String("Context", kubernetesContext))
		if err := client.List(ctx, appRollouts); err != nil {
			slog.ErrorContext(ctx, "Failed to list AppRollouts", ilog.Err(err))
			return false, nil
		}
		slog.InfoContext(ctx, "...done.", slog.Int("AppRolloutCount", len(appRollouts.Items)))

		numBadConditions = 0
		for _, i := range appRollouts.Items {
			ar := &apps.AppRollout{}
			if err := convert(&i, ar); err != nil {
				return false, fmt.Errorf("failed to unmarshall AppRollout: %v", err)
			}
			for _, c := range ar.Status.Conditions {
				slog.InfoContext(ctx, "AppRollout condition", slog.String("Name", ar.GetName()), slog.String("Condition", string(c.Type)), slog.String("Status", string(c.Status)))
				if c.Status != corev1.ConditionTrue {
					slog.WarnContext(ctx, "AppRollout condition not met", slog.String("Name", ar.GetName()), slog.String("Condition", string(c.Type)))
					numBadConditions += 1
				}
			}
		}
		return numBadConditions == 0, nil
	})
	if err != nil {
		if numBadConditions != 0 {
			return fmt.Errorf("unhealthy AppRollout status after waiting for %d sec: %d conditions not met",
				appInitializationTimeout/time.Second, numBadConditions)
		}
		return err
	}
	return nil
}
