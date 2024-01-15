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
	"fmt"
	"log/slog"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
	"strings"

	"github.com/googlecloudrobotics/core/src/go/pkg/configutil"
	"github.com/googlecloudrobotics/core/src/go/pkg/gcr"
	"github.com/googlecloudrobotics/core/src/go/pkg/kubeutils"
	"github.com/googlecloudrobotics/core/src/go/pkg/robotauth"
	"github.com/googlecloudrobotics/core/src/go/pkg/setup"
	"github.com/googlecloudrobotics/ilog"
	"github.com/pkg/errors"
	flag "github.com/spf13/pflag"
	"golang.org/x/oauth2"
	"google.golang.org/api/option"
	corev1 "k8s.io/api/core/v1"
	apierrors "k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/api/validation"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/client-go/dynamic"
	"k8s.io/client-go/kubernetes"
	"k8s.io/client-go/rest"
)

var (
	robotName          = new(string)
	project            = flag.String("project", "", "Project ID for the Google Cloud Platform")
	robotType          = flag.String("robot-type", "", "Robot type. Optional if the robot is already registered.")
	registryID         = flag.String("registry-id", "", "The ID used when writing the public key to the cloud registry. Default: robot-<robot-name>.")
	labels             = flag.String("labels", "", "Robot labels. Optional if the robot is already registered.")
	annotations        = flag.String("annotations", "", "Robot annotations. Optional if the robot is already registered.")
	crSyncer           = flag.Bool("cr-syncer", true, "Set up the cr-syncer, and create a Robot CR in the cloud cluster.")
	fluentd            = flag.Bool("fluentd", true, "Set up fluentd to upload logs to Stackdriver.")
	fluentbit          = flag.Bool("fluentbit", false, "Set up fluentbit to upload logs to Stackdriver.")
	logPrefixSubdomain = flag.String("log-prefix-subdomain", "", "Subdomain to prepend to Fluentbit log tag prefix.")
	dockerDataRoot     = flag.String("docker-data-root", "/var/lib/docker", "This should match data-root in /etc/docker/daemon.json.")
	podCIDR            = flag.String("pod-cidr", "192.168.9.0/24",
		"The range of Pod IP addresses in the cluster. This should match the CNI "+
			"configuration (eg Cilium's clusterPoolIPv4PodCIDR). If this is incorrect, "+
			"pods will get 403 Forbidden when trying to reach the metadata server.")
	robotAuthentication = flag.Bool("robot-authentication", true, "Set up robot authentication.")

	robotGVR = schema.GroupVersionResource{
		Group:    "registry.cloudrobotics.com",
		Version:  "v1alpha1",
		Resource: "robots",
	}
)

const (
	filesDir          = "/setup-robot-files"
	helmPath          = filesDir + "/helm"
	synkPath          = filesDir + "/synk"
	numDNSRetries     = 6
	numServiceRetries = 6
	// commaSentinel is used when parsing labels or annotations.
	commaSentinel = "_COMMA_SENTINEL_"
	baseNamespace = "default"
)

func parseFlags() {
	flag.Usage = func() {
		fmt.Fprintln(os.Stderr, "Usage: setup-robot <robot-name> --project <project-id> [OPTIONS]")
		fmt.Fprintln(os.Stderr, "  robot-name")
		fmt.Fprintln(os.Stderr, "        Robot name")
		fmt.Fprintln(os.Stderr, "")
		flag.PrintDefaults()
	}
	flag.Parse()

	if flag.NArg() < 1 {
		flag.Usage()
		fmt.Println("ERROR: robot-name is required.")
		os.Exit(1)
	} else if flag.NArg() > 1 {
		flag.Usage()
		fmt.Printf("ERROR: too many positional arguments (%d), expected 1.", flag.NArg())
		os.Exit(1)
	} else if errs := validation.NameIsDNS1035Label(flag.Arg(0), false); len(errs) > 0 {
		fmt.Printf("ERROR: invalid cluster name %q: %s", flag.Arg(0), strings.Join(errs, ", "))
		os.Exit(1)
	}

	*robotName = flag.Arg(0)

	if *project == "" {
		flag.Usage()
		fmt.Println("ERROR: --project is required.")
		os.Exit(1)
	}
	if *registryID == "" {
		*registryID = fmt.Sprintf("robot-%s", *robotName)
	}

	if *fluentd && *fluentbit {
		flag.Usage()
		fmt.Println("ERROR: --fluentd and --fluenetbit cannot be enabled at the same time.")
		os.Exit(1)
	}
}

// parseKeyValues splits a string on ',' and the entries on '=' to build a map.
func parseKeyValues(s string) (map[string]string, error) {
	lset := map[string]string{}

	if s == "" {
		return lset, nil
	}

	// To handle escaped commas, we replace them with a sentinel, then
	// restore them after splitting individual values.
	s = strings.ReplaceAll(s, "\\,", commaSentinel)
	for _, l := range strings.Split(s, ",") {
		l = strings.ReplaceAll(l, commaSentinel, ",")
		parts := strings.SplitN(l, "=", 2)
		if len(parts) != 2 {
			return nil, errors.New("not a key/value pair")
		}
		lset[parts[0]] = parts[1]
	}
	return lset, nil
}

// checkRobotName tests whether a Robot resource exists in the local cluster
// with a different name. It is not safe to rerun setup-robot with a different
// name as the chart-assignment-controller doesn't allow the clusterName field to change.
func checkRobotName(ctx context.Context, client dynamic.Interface) error {
	robots, err := client.Resource(robotGVR).Namespace("default").List(ctx, metav1.ListOptions{})
	if err != nil {
		if apierrors.IsNotFound(err) {
			return nil
		}
		return errors.Wrap(err, "list local robots")
	}
	for _, r := range robots.Items {
		if r.GetName() != *robotName {
			return fmt.Errorf(`this cluster was already set up with a different name. It is not safe to rename an existing cluster.
    - either, use the old name:
        setup_robot.sh %q [...]
    - or, reset the cluster before renaming it:
        sudo kubeadm reset
	setup_robot.sh %q [...]`, r.GetName(), *robotName)
		}
	}
	return nil
}

func main() {
	parseFlags()

	logHandler := ilog.NewLogHandler(slog.LevelInfo, os.Stderr)
	slog.SetDefault(slog.New(logHandler))

	ctx := context.Background()
	envToken := os.Getenv("ACCESS_TOKEN")
	if envToken == "" {
		slog.Error("ACCESS_TOKEN environment variable is required.")
		os.Exit(1)
	}
	registry := os.Getenv("REGISTRY")
	if registry == "" {
		slog.Error("REGISTRY environment variable is required.")
		os.Exit(1)
	}
	parsedLabels, err := parseKeyValues(*labels)
	if err != nil {
		slog.Error("Invalid labels", slog.String("Labels", *labels), ilog.Err(err))
		os.Exit(1)
	}
	parsedAnnotations, err := parseKeyValues(*annotations)
	if err != nil {
		slog.Error("Invalid annotations", slog.String("Annotations", *annotations), ilog.Err(err))
		os.Exit(1)
	}

	// Wait for in-cluster DNS to become available, otherwise
	// configutil.ReadConfig() may fail.
	if err := setup.WaitForDNS("storage.googleapis.com", numDNSRetries); err != nil {
		slog.Error("Failed to resolve storage.googleapis.com. Please retry in 5 minutes.", ilog.Err(err))
		os.Exit(1)
	}

	// Set up the OAuth2 token source.
	tokenSource := oauth2.StaticTokenSource(&oauth2.Token{AccessToken: envToken})

	vars, err := configutil.ReadConfig(*project, option.WithTokenSource(tokenSource))
	if err != nil {
		slog.Error("Failed to read config for project", ilog.Err(err))
		os.Exit(1)
	}
	domain, ok := vars["CLOUD_ROBOTICS_DOMAIN"]
	if !ok || domain == "" {
		domain = fmt.Sprintf("www.endpoints.%s.cloud.goog", *project)
	}

	// Wait until we can resolve the project domain. This may require DNS propagation.
	if err := setup.WaitForDNS(domain, numDNSRetries); err != nil {
		slog.Error("Failed to resolve cloud cluster. Please retry in 5 minutes.", ilog.Err(err))
		os.Exit(1)
	}

	// Connect to the surrounding k8s cluster.
	localConfig, err := rest.InClusterConfig()
	if err != nil {
		slog.Error("Failed to load in-cluster config", ilog.Err(err))
		os.Exit(1)
	}
	k8sLocalClientSet, err := kubernetes.NewForConfig(localConfig)
	if err != nil {
		slog.Error("Failed to create kubernetes client set", ilog.Err(err))
		os.Exit(1)
	}
	if _, err := k8sLocalClientSet.AppsV1().Deployments("default").Get(ctx, "app-rollout-controller", metav1.GetOptions{}); err == nil {
		// It's important to avoid deploying the cloud-robotics
		// metadata-server in the same cluster as the token-vendor,
		// otherwise we'll break auth for all robot clusters.
		slog.Error("The local context contains a app-rollout-controller deployment. It is not safe to run robot setup on a GKE cloud cluster.")
		os.Exit(1)
	}
	k8sLocalDynamic, err := dynamic.NewForConfig(localConfig)
	if err != nil {
		slog.Error("Failed to create dynamic client set", ilog.Err(err))
		os.Exit(1)
	}
	if err := checkRobotName(ctx, k8sLocalDynamic); err != nil {
		slog.Error("RobotName", ilog.Err(err))
		os.Exit(1)
	}

	if *robotAuthentication {
		// Set up robot authentication.
		auth := &robotauth.RobotAuth{
			RobotName:           *robotName,
			ProjectId:           *project,
			Domain:              domain,
			PublicKeyRegistryId: *registryID,
		}

		slog.Info("Creating new private key")
		if err := auth.CreatePrivateKey(); err != nil {
			slog.Error("Failed creating key", ilog.Err(err))
			os.Exit(1)
		}
		httpClient := oauth2.NewClient(ctx, tokenSource)
		if err := setup.PublishCredentialsToCloud(httpClient, auth, numServiceRetries); err != nil {
			slog.Error("Failed to publish credentials.", ilog.Err(err))
			os.Exit(1)
		}
		if err := auth.StoreInK8sSecret(ctx, k8sLocalClientSet, baseNamespace); err != nil {
			slog.Error("Failed to write auth secret", ilog.Err(err))
			os.Exit(1)
		}
		if err := gcr.UpdateGcrCredentials(ctx, k8sLocalClientSet, auth); err != nil {
			slog.Error("Failed to update credentials", ilog.Err(err))
			os.Exit(1)
		}
	}

	slog.Info("Initializing Synk")
	output, err := exec.Command(synkPath, "init").CombinedOutput()
	if err != nil {
		slog.Error("Synk init failed.", ilog.Err(err), slog.String("Output", string(output)))
		os.Exit(1)
	}

	appManagement := configutil.GetBoolean(vars, "APP_MANAGEMENT", true)
	// Use "robot" as a suffix for consistency for Synk deployments.
	installChartOrDie(ctx, k8sLocalClientSet, domain, registry, "base-robot", baseNamespace,
		"base-robot-0.0.1.tgz", appManagement)

	// Set up Robot CR as a last step (local CR needs CRD to be deployed)
	if *crSyncer {
		// Set up client for cloud k8s cluster, to create/update the Robot CR there.
		k8sCloudCfg := kubeutils.BuildCloudKubernetesConfig(tokenSource, domain)
		k8sCloudDynamic, err := dynamic.NewForConfig(k8sCloudCfg)
		if err != nil {
			slog.Error("Failed to create k8s client", ilog.Err(err))
			os.Exit(1)
		}
		if err := createOrUpdateRobot(ctx, k8sCloudDynamic, parsedLabels, parsedAnnotations); err != nil {
			slog.Error("Failed to create/update cloud robot CR", slog.String("Name", *robotName), ilog.Err(err))
			os.Exit(1)
		}
	} else {
		// Creating a Robot CR in the cloud would make the app-rollout-controller
		// create ChartAssignments in the cloud, but if the cr-syncer is disabled,
		// these would not be synced/installed.
		// Hence we create a local Robot CR to keep the same interface.
		if err := createOrUpdateRobot(ctx, k8sLocalDynamic, parsedLabels, parsedAnnotations); err != nil {
			slog.Error("Failed to create/update local robot CR", slog.String("Name", *robotName), ilog.Err(err))
			os.Exit(1)
		}
	}

	slog.Info("Setup complete")
}

func helmValuesStringFromMap(varMap map[string]string) string {
	varList := []string{}
	for k, v := range varMap {
		varList = append(varList, fmt.Sprintf("%s=%s", k, v))
	}
	return strings.Join(varList, ",")
}

// installChartOrDie installs a chart using Synk.
func installChartOrDie(ctx context.Context, cs *kubernetes.Clientset, domain, registry, name, namespace, chartPath string, appManagement bool) {
	// ensure namespace for chart exists
	if _, err := cs.CoreV1().Namespaces().Create(ctx,
		&corev1.Namespace{
			ObjectMeta: metav1.ObjectMeta{
				Name: namespace,
			},
		},
		metav1.CreateOptions{}); err != nil && !apierrors.IsAlreadyExists(err) {
		slog.Error("Failed to create namespace.", slog.String("Namespace", namespace), ilog.Err(err))
		os.Exit(1)
	}

	vars := helmValuesStringFromMap(map[string]string{
		"domain":               domain,
		"registry":             registry,
		"project":              *project,
		"app_management":       strconv.FormatBool(appManagement),
		"cr_syncer":            strconv.FormatBool(*crSyncer),
		"fluentd":              strconv.FormatBool(*fluentd),
		"fluentbit":            strconv.FormatBool(*fluentbit),
		"log_prefix_subdomain": *logPrefixSubdomain,
		"docker_data_root":     *dockerDataRoot,
		"pod_cidr":             *podCIDR,
		"robot_authentication": strconv.FormatBool(*robotAuthentication),
		"robot.name":           *robotName,
	})
	slog.Info("Installing chart using Synk",
		slog.String("Chart", name),
		slog.String("Path", chartPath))

	output, err := exec.Command(
		helmPath,
		"template",
		"--set-string", vars,
		"--name", name,
		"--namespace", namespace,
		filepath.Join(filesDir, chartPath),
	).CombinedOutput()
	if err != nil {
		slog.Error("Synk install failed.",
			slog.String("Chart", name),
			ilog.Err(err),
			slog.String("Helm output", string(output)))
		os.Exit(1)
	}
	cmd := exec.Command(
		synkPath,
		"apply",
		name,
		"-n", namespace,
		"-f", "-",
	)
	// Helm writes the templated manifests and errors alike to stderr.
	// So we can just take the combined output as is.
	cmd.Stdin = bytes.NewReader(output)

	if output, err = cmd.CombinedOutput(); err != nil {
		slog.Error("Synk install failed.",
			slog.String("Chart", name),
			ilog.Err(err),
			slog.String("Synk output", string(output)))
		os.Exit(1)
	}
}

func createOrUpdateRobot(ctx context.Context, k8sDynamicClient dynamic.Interface, labels map[string]string, annotations map[string]string) error {
	const masterHost = "cloudrobotics.com/master-host"
	labels["cloudrobotics.com/robot-name"] = *robotName
	host := os.Getenv("HOST_HOSTNAME")
	if host != "" && annotations[masterHost] == "" {
		annotations[masterHost] = host
	}
	crc_version := os.Getenv("CRC_VERSION")
	if crc_version != "" {
		annotations["cloudrobotics.com/crc-version"] = crc_version
	}
	robotClient := k8sDynamicClient.Resource(robotGVR).Namespace("default")
	return setup.CreateOrUpdateRobot(ctx, robotClient, *robotName, *robotType, *project, labels, annotations)
}
