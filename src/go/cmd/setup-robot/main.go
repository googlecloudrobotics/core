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
	"flag"
	"fmt"
	"log"
	"net"
	"net/http"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
	"strings"
	"time"

	"github.com/googlecloudrobotics/core/src/go/pkg/gcr"
	"github.com/googlecloudrobotics/core/src/go/pkg/kubeutils"
	"github.com/googlecloudrobotics/core/src/go/pkg/robotauth"
	"github.com/googlecloudrobotics/core/src/go/pkg/setup"

	"golang.org/x/oauth2"
	"google.golang.org/api/cloudresourcemanager/v1"
	corev1 "k8s.io/api/core/v1"
	rbacv1 "k8s.io/api/rbac/v1"
	apierrors "k8s.io/apimachinery/pkg/api/errors"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/client-go/dynamic"
	"k8s.io/client-go/kubernetes"
	"k8s.io/client-go/rest"
)

var (
	domain              = flag.String("domain", "", "Domain for the Cloud Robotics project (default: www.endpoints.<project>.cloud.goog)")
	project             = flag.String("project", "", "Project ID for the Google Cloud Platform")
	robotName           = flag.String("robot-name", "", "Robot name")
	robotRole           = flag.String("robot-role", "", "Robot role. Optional if the robot is already registered.")
	robotType           = flag.String("robot-type", "", "Robot type. Optional if the robot is already registered.")
	robotAuthentication = flag.Bool("robot-authentication", true, "Set up robot authentication.")
	appManagement       = flag.Bool("app-management", true, "Set up app management.")
)

const (
	filesDir      = "/setup-robot-files"
	helmPath      = filesDir + "/helm"
	numDNSRetries = 6
)

func getProjectNumber(client *http.Client, projectID string) (int64, error) {
	crm, err := cloudresourcemanager.New(client)
	if err != nil {
		return 0, err
	}
	project, err := crm.Projects.Get(projectID).Do()
	if err != nil {
		return 0, err
	}
	return project.ProjectNumber, nil
}

func parseFlags() {
	flag.Parse()

	if *project == "" {
		log.Fatal("--project is required.")
	}
	if *robotName == "" {
		log.Fatal("--robot-name is required.")
	}
	if *domain == "" {
		*domain = fmt.Sprintf("www.endpoints.%s.cloud.goog", *project)
	}
}

// Since this might be the first interaction with the cluster, manually resolve the
// domain name with retries to give a better error in the case of failure.
func waitForDNS() error {
	log.Printf("DNS lookup for %q", *domain)
	delay := time.Second
	for i := 0; i < numDNSRetries; i++ {
		ips, err := net.LookupIP(*domain)
		if err == nil {
			// Check that the results contain an ipv4 addr. Initially, coredns may only
			// return ipv6 addresses in which case helm will fail.
			for _, ip := range ips {
				if ip.To4() != nil {
					return nil
				}
			}
		}
		log.Printf("... Retry dns for %q", *domain)
		time.Sleep(delay)
		delay += delay
	}
	return fmt.Errorf("DNS lookup for %q failed", *domain)
}

func main() {
	parseFlags()
	envToken := os.Getenv("ACCESS_TOKEN")
	if envToken == "" {
		log.Fatal("ACCESS_TOKEN environment variable is required.")
	}

	if err := waitForDNS(); err != nil {
		log.Fatalf("Failed to resolve cloud cluster: %s. Please retry in 5 minutes.", err)
	}

	// Set up the OAuth2 token source and client.
	tokenSource := oauth2.StaticTokenSource(&oauth2.Token{AccessToken: envToken})
	client := oauth2.NewClient(context.Background(), tokenSource)

	if *robotRole != "" || *robotType != "" {
		if err := createOrUpdateRobot(tokenSource); err != nil {
			log.Fatalf("Failed to update robot CR %v: %v", *robotName, err)
		}
	}

	// Connect to the surrounding k8s cluster.
	localConfig, err := rest.InClusterConfig()
	if err != nil {
		log.Fatal("Failed to load in-cluster config: ", err)
	}
	k8sLocalClientSet, err := kubernetes.NewForConfig(localConfig)
	if err != nil {
		log.Fatal("Failed to create kubernetes client set: ", err)
	}

	if *robotAuthentication {
		// Set up robot authentication.
		auth := &robotauth.RobotAuth{
			RobotName:           *robotName,
			ProjectId:           *project,
			Domain:              *domain,
			PublicKeyRegistryId: fmt.Sprintf("robot-%s", *robotName),
		}
		if err := setup.CreateAndPublishCredentialsToCloud(client, auth); err != nil {
			log.Fatal(err)
		}
		if err := auth.StoreInK8sSecret(k8sLocalClientSet); err != nil {
			log.Fatal(fmt.Errorf("Failed to write auth secret: %v", err))
		}
		if err := gcr.UpdateGcrCredentials(k8sLocalClientSet, auth); err != nil {
			log.Fatal(err)
		}
	}

	// Get the project number, which is passed as a value to the helm charts.
	projectNumber, err := getProjectNumber(client, *project)
	if err != nil {
		log.Fatalf("Failed to get project number: %v", err)
	}

	// Create service account and role binding for Tiller.
	// (this isn't strictly necessary until we're using auth properly, but it's
	//  one less thing to fix when RBAC is used properly)
	if _, err := k8sLocalClientSet.CoreV1().ServiceAccounts("kube-system").Create(&corev1.ServiceAccount{
		ObjectMeta: metav1.ObjectMeta{
			Name:      "tiller",
			Namespace: "kube-system",
		},
	}); err != nil {
		log.Println("Failed to create tiller service account: ", err)
	}
	if _, err := k8sLocalClientSet.RbacV1().ClusterRoleBindings().Create(&rbacv1.ClusterRoleBinding{
		ObjectMeta: metav1.ObjectMeta{
			Name:      "tiller",
			Namespace: "kube-system",
		},
		RoleRef: rbacv1.RoleRef{
			APIGroup: "rbac.authorization.k8s.io",
			Kind:     "ClusterRole",
			Name:     "cluster-admin",
		},
		Subjects: []rbacv1.Subject{{
			Kind:      "ServiceAccount",
			Name:      "tiller",
			Namespace: "kube-system",
		}},
	}); err != nil {
		log.Println("Failed to create tiller role binding: ", err)
	}

	log.Println("Initializing Helm")
	output, err := exec.Command(
		helmPath,
		"init",
		"--history-max=10",
		"--upgrade",
		"--force-upgrade",
		"--wait",
		"--service-account=tiller").CombinedOutput()
	if err != nil {
		log.Fatalf("Helm init failed: %v. Helm output:\n%s\n", err, output)
	}

	// Clean up deprecated releases.
	deleteReleaseIfPresent("robot-cluster")

	installChartOrDie("robot-base", "base-robot-0.0.1.tgz", projectNumber)
}

func helmValuesStringFromMap(varMap map[string]string) string {
	varList := []string{}
	for k, v := range varMap {
		varList = append(varList, fmt.Sprintf("%s=%s", k, v))
	}
	return strings.Join(varList, ",")
}

func deleteReleaseIfPresent(name string) {
	output, err := exec.Command(
		helmPath,
		"delete",
		"--purge",
		name).CombinedOutput()
	if err != nil {
		log.Printf("Helm delete of %s failed: %v. Helm output:\n%s\n", name, err, output)
	} else {
		log.Printf("%s\nSuccessfully removed %s Helm release", output, name)
	}
}

func installChartOrDie(name string, chartPath string, projectNumber int64) {
	log.Printf("Installing %s Helm chart from %s", name, chartPath)
	vars := helmValuesStringFromMap(map[string]string{
		"domain":               *domain,
		"project":              *project,
		"project_number":       strconv.FormatInt(projectNumber, 10),
		"app_management":       strconv.FormatBool(*appManagement),
		"robot_authentication": strconv.FormatBool(*robotAuthentication),
		"robot.name":           *robotName,
	})
	output, err := exec.Command(
		helmPath,
		"upgrade",
		"--install",
		"--force",
		"--set-string",
		vars,
		name,
		filepath.Join(filesDir, chartPath)).CombinedOutput()
	if err != nil {
		log.Fatalf("Helm install of %s failed: %v. Helm output:\n%s\n", name, err, output)
	} else {
		log.Printf("%s\nSuccessfully installed %s Helm chart", output, name)
	}
}

func createOrUpdateRobot(tokenSource oauth2.TokenSource) error {
	// Set up client for cloud k8s cluster (needed only to obtain list of robots).
	k8sCloudCfg := kubeutils.BuildCloudKubernetesConfig(tokenSource, *domain)
	k8sDynamicClient, err := dynamic.NewForConfig(k8sCloudCfg)
	if err != nil {
		return err
	}
	robotGVR := schema.GroupVersionResource{
		Group:    "registry.cloudrobotics.com",
		Version:  "v1alpha1",
		Resource: "robots"}
	robotClient := k8sDynamicClient.Resource(robotGVR).Namespace("default")
	robot, err := robotClient.Get(*robotName, metav1.GetOptions{})
	if err != nil {
		if s, ok := err.(*apierrors.StatusError); ok && s.ErrStatus.Reason == metav1.StatusReasonNotFound {
			robot := &unstructured.Unstructured{}
			robot.SetKind("Robot")
			robot.SetAPIVersion("registry.cloudrobotics.com/v1alpha1")
			robot.SetName(*robotName)
			robot.SetLabels(map[string]string{"cloudrobotics.com/robot-name": *robotName})
			robot.Object["spec"] = map[string]string{
				"role":    *robotRole,
				"type":    *robotType,
				"project": *project,
			}
			robot.Object["status"] = make(map[string]interface{})
			_, err := robotClient.Create(robot, metav1.CreateOptions{})
			return err
		} else {
			return fmt.Errorf("Failed to get robot %v: %v", *robotName, err)
		}
	}
	spec, ok := robot.Object["spec"].(map[string]interface{})
	if !ok {
		return fmt.Errorf("unmarshaling robot failed: spec is not a map")
	}
	spec["role"] = *robotRole
	spec["type"] = *robotType
	spec["project"] = *project
	_, err = robotClient.Update(robot, metav1.UpdateOptions{})
	return err
}
