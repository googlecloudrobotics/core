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

// Synchronize the k8s cluster to the desired state described in the database.
//
// Whenever robots get added or deleted in the database, the master updated the
// k8s cluster accordingly.
package main

import (
	"context"
	"fmt"

	pb "src/proto/registry"

	"github.com/googlecloudrobotics/core/src/go/pkg/helm"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/client-go/dynamic"
	"k8s.io/client-go/rest"
)

type cloudMaster struct {
	h         *helm.Helm
	k8sClient dynamic.Interface
}

func newCloudMaster(kubernetesConfig *rest.Config) (*cloudMaster, error) {
	k8sClient, err := dynamic.NewForConfig(kubernetesConfig)
	if err != nil {
		return nil, fmt.Errorf("failed to initialize Kubernetes client: %v", err)
	}

	h, err := helm.NewHelm(k8sClient)
	if err != nil {
		return nil, err
	}

	return &cloudMaster{h, k8sClient}, nil
}

func (m *cloudMaster) spin(ctx context.Context) error {
	robots, err := m.listRobots()
	if err != nil {
		return fmt.Errorf("failed to list robots: %v", err)
	}

	if err := m.h.InstallApps(ctx, "cloud-robots", pb.InstallationTarget_CLOUD_PER_ROBOT, robots); err != nil {
		return fmt.Errorf("failed to install cloud-per-robot chart: %v", err)
	}

	if err := m.h.InstallApps(ctx, "cloud-apps", pb.InstallationTarget_CLOUD, robots); err != nil {
		return fmt.Errorf("failed to install cloud-apps chart: %v", err)
	}

	return nil
}

func (m *cloudMaster) listRobots() ([]unstructured.Unstructured, error) {
	robotGVR := schema.GroupVersionResource{Group: "registry.cloudrobotics.com", Version: "v1alpha1", Resource: "robots"}
	robotClient := m.k8sClient.Resource(robotGVR).Namespace("default")

	listResponse, err := robotClient.List(metav1.ListOptions{})
	if err != nil {
		return nil, fmt.Errorf("failed to list robot CRs: %v", err)
	}
	return listResponse.Items, nil
}
