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

/*
Helm is a k8s package manager. We are only using a subset of its features. We're not yet using
chart repositories. Instead we build a chart on-the-fly and use that to upgrade the cluster.

For templating the charts we provide the following values:

    robot:
      id: <Robot.ID>
      name: <Robot.DisplayName>
      type: <Robot.Type>
      role: <Robot.Role>
    app:
      sim_suffix: {'','-sim'}
      role: {'nav','slam',....}
*/
package helm

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/json"
	"fmt"
	"io/ioutil"
	"log"
	"os"
	"os/exec"
	"path/filepath"
	"sort"

	pb "src/proto/registry"

	"github.com/golang/protobuf/jsonpb"
	"github.com/golang/protobuf/proto"
	yaml "gopkg.in/yaml.v2"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/apimachinery/pkg/watch"
	"k8s.io/client-go/dynamic"
	"k8s.io/client-go/tools/cache"
)

type Helm struct {
	lastPushChecksum map[string][]byte
	helmBinary       string
	helmParams       map[string]interface{}
	releaseName      string
	roleStore        cache.Store
	appStore         cache.Store
	trigger          <-chan struct{}
}

func buildSharedInformer(client dynamic.ResourceInterface, trigger chan<- struct{}) cache.SharedInformer {
	done := context.Background().Done()
	informer := cache.NewSharedInformer(
		&cache.ListWatch{
			ListFunc: func(options metav1.ListOptions) (runtime.Object, error) {
				return client.List(options)
			},
			WatchFunc: func(options metav1.ListOptions) (watch.Interface, error) {
				return client.Watch(options)
			},
		},
		&unstructured.Unstructured{},
		0)

	go informer.Run(done)

	if !cache.WaitForCacheSync(done, informer.HasSynced) {
		log.Fatalf("WaitForCacheSync for roles failed")
	}

	sendNonBlockingTrigger := func() {
		select {
		case trigger <- struct{}{}:
		default:
		}
	}
	informer.AddEventHandler(cache.ResourceEventHandlerFuncs{
		AddFunc: func(obj interface{}) {
			o := obj.(*unstructured.Unstructured)
			log.Printf("Got add event for %s %s", o.GetKind(), o.GetName())
			sendNonBlockingTrigger()
		},
		UpdateFunc: func(oldObj, newObj interface{}) {
			// For some reason, we get a lot of no-op update events from the informer. Maybe because we're accessing it's underlying store.
			o := oldObj.(*unstructured.Unstructured)
			no := newObj.(*unstructured.Unstructured)
			if o.GetResourceVersion() != no.GetResourceVersion() {
				log.Printf("Got update event for %s %s", o.GetKind(), o.GetName())
				sendNonBlockingTrigger()
			}
		},
		DeleteFunc: func(obj interface{}) {
			o := obj.(*unstructured.Unstructured)
			log.Printf("Got delete event for %s %s", o.GetKind(), o.GetName())
			sendNonBlockingTrigger()
		},
	})

	return informer
}

func NewHelm(k8s dynamic.Interface, params map[string]interface{}) (*Helm, error) {
	roleGVR := schema.GroupVersionResource{Group: "registry.cloudrobotics.com", Version: "v1alpha1", Resource: "roles"}
	appGVR := schema.GroupVersionResource{Group: "registry.cloudrobotics.com", Version: "v1alpha1", Resource: "apps"}
	roleClient := k8s.Resource(roleGVR).Namespace("default")
	appClient := k8s.Resource(appGVR).Namespace("default")
	// We make trigger a buffered channel of size 1 to deduplicate events.
	// As long as the listener hasn't retrieved the current event, we don't
	// send new events to the channel by using a non-blocking send.
	trigger := make(chan struct{}, 1)
	roleInformer := buildSharedInformer(roleClient, trigger)
	appInformer := buildSharedInformer(appClient, trigger)

	return &Helm{
		lastPushChecksum: make(map[string][]byte),
		helmBinary:       "/helm",
		helmParams:       params,
		roleStore:        roleInformer.GetStore(),
		appStore:         appInformer.GetStore(),
		trigger:          trigger,
	}, nil
}

// InstallApps installs all apps for the robots on the cluster, and uninstalls
// any apps not in the list. It builds a synthetic chart on local disk that
// depends on all the charts for robot apps, and installs the chart as a release
// with a static name. The release name must stay the same over the lifetime of
// the Kubernetes cluster.
// Using the synthetic chart solves the problem of tracking apps that need to be
// installed and uninstalling apps that should not be.
func (h *Helm) InstallApps(ctx context.Context, releaseName, namespace string, target pb.InstallationTarget, robots []unstructured.Unstructured) error {
	var mc *masterChart
	var err error
	if target == pb.InstallationTarget_CLOUD {
		mc, err = h.buildCloudChart()
	} else {
		mc, err = h.buildPerRobotChart(robots, target)
	}
	if err != nil {
		return fmt.Errorf("failed to assemble chart: %v", err)
	}
	// Sort requirements.yaml by alias to stabilize checksums. values.yaml
	// is a dict anyway, so it's already has a canonical YAML serialization.
	sort.Slice(mc.requirements.Dependencies, func(i, j int) bool {
		return mc.requirements.Dependencies[i].Alias < mc.requirements.Dependencies[j].Alias
	})

	checksum, err := h.checksumChart(ctx, mc)
	if err != nil {
		return fmt.Errorf("failed to checksum subcharts: %v", err)
	}
	if bytes.Equal(checksum, h.lastPushChecksum[releaseName]) {
		log.Printf("Chart for release %s is identical to last push, eliding push.", releaseName)
		return nil
	}

	chartDir, err := ioutil.TempDir("", "synthetic-chart")
	if err != nil {
		return fmt.Errorf("failed to create temp dir for chart: %v", err)
	}
	defer os.RemoveAll(chartDir)
	if err := setUpChart(chartDir, mc); err != nil {
		return err
	}
	if err := h.installDependencies(ctx, chartDir, mc); err != nil {
		return err
	}
	// TODO(b/116459365): double the default timeout as a workaround
	args := []string{
		"upgrade", "--install", "--force", "--namespace", namespace, "--timeout", "600",
		releaseName,
		chartDir,
	}
	log.Printf("Updating chart for release %s. Applying to %d robots.", releaseName,
		len(robots))
	output, err := exec.Command(h.helmBinary, args...).CombinedOutput()
	if err != nil {
		delete(h.lastPushChecksum, releaseName)
		log.Printf("Helm failed with %v:\n%s", args, output)
		return err
	}
	h.lastPushChecksum[releaseName] = checksum
	log.Printf("Helm succeeded with %v:\n%s", args, output)
	return nil
}

// The channel returned by UpdateChannel emits an event when apps or roles were changed.
func (h *Helm) UpdateChannel() <-chan struct{} {
	return h.trigger
}

type masterChart struct {
	requirements requirements
	values       map[string]interface{}
	inlineCharts map[string][]byte
}

type dependency struct {
	Name    string
	Version string
	Alias   string
}

type requirements struct {
	Dependencies []dependency
}

// A config map to ensure the helm chart is never empty.
var helmStatusTemplate = `
kind: ConfigMap
apiVersion: v1
metadata:
  name: helm-status-{{ .Release.Name }}
data:
  release: {{ .Release.Name }}
`

func getSpecProto(obj unstructured.Unstructured, msg proto.Message) error {
	b, err := json.Marshal(obj.Object["spec"])
	if err != nil {
		return err
	}
	err = jsonpb.Unmarshal(bytes.NewReader(b), msg)
	if err != nil {
		return err
	}
	return nil
}

func (h *Helm) getMatchingRoles(roleDisplayName string) (map[string]*pb.RoleSpec, error) {
	// TODO(swolter): Consider caching.
	result := make(map[string]*pb.RoleSpec)
	for _, roleObj := range h.roleStore.List() {
		role := *roleObj.(*unstructured.Unstructured)
		roleSpec := &pb.RoleSpec{}
		if err := getSpecProto(role, roleSpec); err != nil {
			return nil, err
		}
		// DisplayName is a poor man's selector block: All roles with the same display
		// name apply. See registry.proto for more details.
		if roleSpec.DisplayName == roleDisplayName {
			result[role.GetName()] = roleSpec
		}
	}
	return result, nil
}

func (h *Helm) getChart(appName, roleName string, target pb.InstallationTarget) (*pb.HelmChart, error) {
	// TODO(swolter): Consider caching.
	appObj, exists, err := h.appStore.GetByKey("default/" + appName)
	if err != nil {
		return nil, err
	}
	if !exists {
		log.Printf("App %s referenced in role %s does not exist, skipping...", appName, roleName)
		return nil, nil
	}
	app := *appObj.(*unstructured.Unstructured)
	appSpec := pb.AppSpec{}
	if err := getSpecProto(app, &appSpec); err != nil {
		return nil, err
	}
	for _, chart := range appSpec.Charts {
		if chart.InstallationTarget == target {
			return chart, nil
		}
	}
	return nil, nil
}

func writeAsYAML(chartDir string, fileName string, values interface{}) error {
	valuesYaml, err := yaml.Marshal(values)
	if err != nil {
		return fmt.Errorf("unable to marshal values: %v", err)
	}
	if err := ioutil.WriteFile(filepath.Join(chartDir, fileName), valuesYaml, 0400); err != nil {
		return fmt.Errorf("unable to write %s: %v", fileName, err)
	}
	return nil
}

// setUpChart sets up a Helm chart in the directory.
func setUpChart(chartDir string, mc *masterChart) error {
	// This name is the chart directory name, not the releaseName.
	chartYaml := "name: cloud-robots\nversion: 0.0.1\n"
	if err := ioutil.WriteFile(filepath.Join(chartDir, "Chart.yaml"), []byte(chartYaml), 0400); err != nil {
		return fmt.Errorf("unable to write Chart.yaml: %v", err)
	}
	if err := os.Mkdir(filepath.Join(chartDir, "charts"), 0700); err != nil {
		return fmt.Errorf("unable to create charts subdirectory: %v", err)
	}
	if err := os.Mkdir(filepath.Join(chartDir, "templates"), 0700); err != nil {
		return fmt.Errorf("unable to create templates subdirectory: %v", err)
	}
	// Add a configmap to handle the case where we have zero robots.
	if err := ioutil.WriteFile(filepath.Join(chartDir, "templates", "helm-status.yaml"),
		[]byte(helmStatusTemplate), 0400); err != nil {
		return fmt.Errorf("unable to write helm-status.yaml: %v", err)
	}

	if err := writeAsYAML(chartDir, "values.yaml", mc.values); err != nil {
		return fmt.Errorf("failed to generate values.yaml: %v", err)
	}
	if err := writeAsYAML(chartDir, "requirements.yaml", mc.requirements); err != nil {
		return fmt.Errorf("failed to generate requirements.yaml: %v", err)
	}
	return nil
}

// merge merges two generic JSON structures giving preference to the values from b.
func merge(a, b map[string]interface{}) map[string]interface{} {
	result := make(map[string]interface{})
	for key, bValue := range b {
		aValue := a[key]
		switch bMap := bValue.(type) {
		case map[string]interface{}:
			aMap, ok := aValue.(map[string]interface{})
			if ok {
				result[key] = merge(aMap, bMap)
			} else {
				result[key] = bValue
			}
		default:
			result[key] = bValue
		}
	}
	for key, aValue := range a {
		if _, ok := b[key]; !ok {
			result[key] = aValue
		}
	}
	return result
}

// buildPerRobotChart adds one subchart for each robot/app combination.
func (h *Helm) buildPerRobotChart(robots []unstructured.Unstructured, target pb.InstallationTarget) (*masterChart, error) {
	result := &masterChart{
		values:       make(map[string]interface{}),
		inlineCharts: make(map[string][]byte),
	}

	for _, r := range robots {
		spec, ok := r.Object["spec"].(map[string]interface{})
		if !ok {
			return nil, fmt.Errorf("unmarshaling robot %q failed: spec is not a map", r.GetName())
		}
		robotType, ok := spec["type"].(string)
		if !ok {
			return nil, fmt.Errorf("unmarshaling robot %q failed: type is not a string", r.GetName())
		}
		robotRole, ok := spec["role"].(string)
		if !ok {
			return nil, fmt.Errorf("unmarshaling robot %q failed: role is not a string", r.GetName())
		}
		robotValues := map[string]interface{}{
			"name": r.GetName(),
			"type": robotType,
			"role": robotRole,
		}
		roles, err := h.getMatchingRoles(robotRole)
		if err != nil {
			return nil, fmt.Errorf("getting the apps for role %q failed: %v", robotRole, err)
		}
		for roleName, roleSpec := range roles {
			for _, appSetting := range roleSpec.Apps {
				chart, err := h.getChart(appSetting.App, roleName, target)
				if err != nil {
					return nil, fmt.Errorf("reading app %s failed: %v", appSetting.App, err)
				}
				if chart == nil {
					continue
				}
				alias := fmt.Sprintf("robot-%s-app-%s", r.GetName(), appSetting.App)
				baseValues := map[string]interface{}{
					"robot": robotValues,
					"app":   appSetting.Settings,
				}
				appValues := make(map[string]interface{})
				if err := yaml.Unmarshal([]byte(chart.Values), &appValues); err != nil {
					return nil, fmt.Errorf("unable to parse values in app %s: %v", appSetting.App, err)
				}
				roleValues := make(map[string]interface{})
				if err := yaml.Unmarshal([]byte(appSetting.Values), &roleValues); err != nil {
					return nil, fmt.Errorf("unable to parse values in role %s: %v", roleName, err)
				}
				result.values[alias] = merge(merge(merge(h.helmParams, baseValues), appValues), roleValues)
				result.requirements.Dependencies = append(result.requirements.Dependencies, dependency{
					Name:    chart.Name,
					Version: chart.Version,
					Alias:   alias,
				})
				if chart.InlineChart == nil {
					return nil, fmt.Errorf("chart %s in app %s is missing inline_chart data", chart.Name, appSetting.App)
				}
				result.inlineCharts[alias] = chart.InlineChart
			}
		}
	}
	return result, nil
}

// buildCloudChart adds one subchart for each app that has a chart and is in a robot's role.
func (h *Helm) buildCloudChart() (*masterChart, error) {
	result := &masterChart{
		values:       make(map[string]interface{}),
		inlineCharts: make(map[string][]byte),
	}

	for _, appObj := range h.appStore.List() {
		app := appObj.(*unstructured.Unstructured)
		alias := app.GetName()
		appSpec := pb.AppSpec{}
		if err := getSpecProto(*app, &appSpec); err != nil {
			return nil, err
		}
		for _, chart := range appSpec.Charts {
			if chart.InstallationTarget != pb.InstallationTarget_CLOUD {
				continue
			}
			log.Printf("Installing cloud chart %s (%s) for app %s", chart.Name, chart.Version, alias)
			result.values[alias] = merge(h.helmParams, map[string]interface{}{})
			if err := yaml.Unmarshal([]byte(chart.Values), result.values[alias]); err != nil {
				return nil, fmt.Errorf("unable to parse values in app %s: %v", alias, err)
			}
			result.requirements.Dependencies = append(result.requirements.Dependencies, dependency{
				Name:    chart.Name,
				Version: chart.Version,
				Alias:   alias,
			})
			if chart.InlineChart == nil {
				return nil, fmt.Errorf("chart %s in app %s is missing inline_chart data", chart.Name, alias)
			}
			result.inlineCharts[alias] = chart.InlineChart
		}
	}
	return result, nil
}

// checksumChart computes a checksum over all subcharts of chart.
func (h *Helm) checksumChart(ctx context.Context, mc *masterChart) ([]byte, error) {
	valuesYaml, err := yaml.Marshal(mc.values)
	if err != nil {
		return nil, fmt.Errorf("unable to marshal values")
	}
	requirementsYaml, err := yaml.Marshal(mc.requirements)
	if err != nil {
		return nil, fmt.Errorf("unable to marshal requirements")
	}
	fingerprint := append(valuesYaml, requirementsYaml...)
	chartAliases := make([]string, len(mc.requirements.Dependencies))
	for index, dep := range mc.requirements.Dependencies {
		chartAliases[index] = dep.Alias
	}
	sort.Strings(chartAliases)

	for _, alias := range chartAliases {
		fingerprint = append(fingerprint, mc.inlineCharts[alias]...)
	}
	shasum := sha256.Sum256(fingerprint)
	return shasum[:], nil
}

// installDependencies downloads all subcharts from requirements.yaml.
// This function is the moral equivalent of "helm dependency update" command.
// Currently, we install all charts from local disk (which helm dependency
// update can't do), so we hand-code the function.
func (h *Helm) installDependencies(ctx context.Context, chartDir string, mc *masterChart) error {
	for _, dep := range mc.requirements.Dependencies {
		chartName := fmt.Sprintf("%s-%s.tgz", dep.Name, dep.Version)
		if err := ioutil.WriteFile(filepath.Join(chartDir, "charts", chartName), mc.inlineCharts[dep.Alias], 0644); err != nil {
			return fmt.Errorf("unable to create chart file %s: %v", chartName, err)
		}
	}

	return nil
}
