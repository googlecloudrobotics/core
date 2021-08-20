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
	"crypto/rand"
	"crypto/rsa"
	"crypto/x509"
	"crypto/x509/pkix"
	b64 "encoding/base64"
	"encoding/json"
	"encoding/pem"
	"fmt"
	"log"
	"math/big"
	"net"
	"net/http"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
	"strings"
	"time"

	"github.com/cenkalti/backoff"
	"github.com/googlecloudrobotics/core/src/go/pkg/configutil"
	"github.com/googlecloudrobotics/core/src/go/pkg/gcr"
	"github.com/googlecloudrobotics/core/src/go/pkg/kubeutils"
	"github.com/googlecloudrobotics/core/src/go/pkg/robotauth"
	"github.com/googlecloudrobotics/core/src/go/pkg/setup"
	"github.com/pkg/errors"
	flag "github.com/spf13/pflag"
	"golang.org/x/oauth2"
	"google.golang.org/api/option"
	corev1 "k8s.io/api/core/v1"
	apierrors "k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/api/validation"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/client-go/dynamic"
	"k8s.io/client-go/kubernetes"
	"k8s.io/client-go/rest"
)

var (
	robotName           = new(string)
	project             = flag.String("project", "", "Project ID for the Google Cloud Platform")
	robotType           = flag.String("robot-type", "", "Robot type. Optional if the robot is already registered.")
	registryID          = flag.String("registry-id", "", "The ID used when writing the public key to the cloud registry. Default: robot-<robot-name>.")
	labels              = flag.String("labels", "", "Robot labels. Optional if the robot is already registered.")
	annotations         = flag.String("annotations", "", "Robot annotations. Optional if the robot is already registered.")
	crSyncer            = flag.Bool("cr-syncer", true, "Set up the cr-syncer.")
	fluentd             = flag.Bool("fluentd", true, "Set up fluentd to upload logs to Stackdriver.")
	dockerDataRoot      = flag.String("docker-data-root", "/var/lib/docker", "This should match data-root in /etc/docker/daemon.json.")
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
		log.Fatal("ERROR: robot-name is required.")
	} else if flag.NArg() > 1 {
		flag.Usage()
		log.Fatalf("ERROR: too many positional arguments (%d), expected 1.", flag.NArg())
	} else if errs := validation.ValidateClusterName(flag.Arg(0), false); len(errs) > 0 {
		log.Fatalf("ERROR: invalid cluster name %q: %s", flag.Arg(0), strings.Join(errs, ", "))
	}

	*robotName = flag.Arg(0)

	if *project == "" {
		flag.Usage()
		log.Fatal("ERROR: --project is required.")
	}
	if *registryID == "" {
		*registryID = fmt.Sprintf("robot-%s", *robotName)
	}
}

func newExponentialBackoff(initialInterval time.Duration, multiplier float64, retries uint64) backoff.BackOff {
	exponentialBackoff := backoff.ExponentialBackOff{
		InitialInterval: initialInterval,
		Multiplier:      multiplier,
		Clock:           backoff.SystemClock,
	}
	exponentialBackoff.Reset()
	return backoff.WithMaxRetries(&exponentialBackoff, retries)
}

// Since this might be the first interaction with the cluster, manually resolve the
// domain name with retries to give a better error in the case of failure.
func waitForDNS(domain string, retries uint64) error {
	log.Printf("DNS lookup for %q", domain)

	if err := backoff.RetryNotify(
		func() error {
			ips, err := net.LookupIP(domain)
			if err != nil {
				return err
			}

			// Check that the results contain an ipv4 addr. Initially, coredns may only
			// return ipv6 addresses in which case helm will fail.
			for _, ip := range ips {
				if ip.To4() != nil {
					return nil
				}
			}

			return errors.New("IP not found")
		},
		newExponentialBackoff(time.Second, 2, retries),
		func(_ error, _ time.Duration) {
			log.Printf("... Retry dns for %q", domain)
		},
	); err != nil {
		return fmt.Errorf("DNS lookup for %q failed: %v", domain, err)
	}

	return nil
}

// Tests a given cloud endpoint with a HEAD request a few times. This lets us wait for the service
// to be available or error with a better message
func waitForService(client *http.Client, url string, retries uint64) error {
	log.Printf("Service probe for %q", url)

	if err := backoff.RetryNotify(
		func() error {
			_, err := client.Head(url)
			return err
		},
		newExponentialBackoff(time.Second, 2, retries),
		func(_ error, _ time.Duration) {
			log.Printf("... Retry service for %q", url)
		},
	); err != nil {
		return fmt.Errorf("service probe for %q failed: %v", url, err)
	}

	return nil
}

// checkRobotName tests whether a Robot resource exists in the local cluster
// with a different name. It is not safe to rerun setup-robot with a different
// name as the robot-master doesn't allow the clusterName field to change.
func checkRobotName(client dynamic.Interface) error {
	robots, err := client.Resource(robotGVR).Namespace("default").List(metav1.ListOptions{})
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

// storeInK8sSecret write new robot-id to kubernetes secret.
func storeInK8sSecret(clientset *kubernetes.Clientset, namespace string, r *robotauth.RobotAuth) error {
	authJson, err := json.Marshal(r)
	if err != nil {
		return err
	}

	return kubeutils.UpdateSecret(
		clientset,
		"robot-auth",
		namespace,
		corev1.SecretTypeOpaque,
		map[string][]byte{
			"json": authJson,
		})
}

func main() {
	parseFlags()
	envToken := os.Getenv("ACCESS_TOKEN")
	if envToken == "" {
		log.Fatal("ACCESS_TOKEN environment variable is required.")
	}
	registry := os.Getenv("REGISTRY")
	if registry == "" {
		log.Fatal("REGISTRY environment variable is required.")
	}
	parsedLabels, err := parseKeyValues(*labels)
	if err != nil {
		log.Fatalf("Invalid labels %q: %s", *labels, err)
	}
	parsedAnnotations, err := parseKeyValues(*annotations)
	if err != nil {
		log.Fatalf("Invalid annotations %q: %s", *annotations, err)
	}

	// Wait for in-cluster DNS to become available, otherwise
	// configutil.ReadConfig() may fail.
	if err := waitForDNS("storage.googleapis.com", numDNSRetries); err != nil {
		log.Fatalf("Failed to resolve storage.googleapis.com: %s. Please retry in 5 minutes.", err)
	}

	// Set up the OAuth2 token source.
	tokenSource := oauth2.StaticTokenSource(&oauth2.Token{AccessToken: envToken})

	vars, err := configutil.ReadConfig(*project, option.WithTokenSource(tokenSource))
	if err != nil {
		log.Fatal("Failed to read config for project: ", err)
	}
	domain, ok := vars["CLOUD_ROBOTICS_DOMAIN"]
	if !ok || domain == "" {
		domain = fmt.Sprintf("www.endpoints.%s.cloud.goog", *project)
	}

	// Wait until we can resolve the project domain. This may require DNS propagation.
	if err := waitForDNS(domain, numDNSRetries); err != nil {
		log.Fatalf("Failed to resolve cloud cluster: %s. Please retry in 5 minutes.", err)
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
	if _, err := k8sLocalClientSet.AppsV1().Deployments("default").Get("cloud-master", metav1.GetOptions{}); err == nil {
		// It's important to avoid deploying the cloud-robotics
		// metadata-server in the same cluster as the token-vendor,
		// otherwise we'll break auth for all robot clusters.
		log.Fatal("The local context contains a cloud-master deployment. It is not safe to run robot setup on a GKE cloud cluster.")
	}
	k8sLocalDynamic, err := dynamic.NewForConfig(localConfig)
	if err != nil {
		log.Fatal("Failed to create dynamic client set: ", err)
	}
	if err := checkRobotName(k8sLocalDynamic); err != nil {
		log.Fatal("Error: ", err)
	}

	if *robotType != "" || *labels != "" || *annotations != "" {
		// Set up client for cloud k8s cluster (needed only to obtain list of robots).
		k8sCloudCfg := kubeutils.BuildCloudKubernetesConfig(tokenSource, domain)
		k8sDynamicClient, err := dynamic.NewForConfig(k8sCloudCfg)
		if err != nil {
			log.Fatalf("Failed to create k8s client: %v", err)
		}
		if err := createOrUpdateRobot(k8sDynamicClient, parsedLabels, parsedAnnotations); err != nil {
			log.Fatalf("Failed to update robot CR %v: %v", *robotName, err)
		}
	}

	httpClient := oauth2.NewClient(context.Background(), tokenSource)

	if *robotAuthentication {
		// Set up robot authentication.
		auth := &robotauth.RobotAuth{
			RobotName:           *robotName,
			ProjectId:           *project,
			Domain:              domain,
			PublicKeyRegistryId: *registryID,
		}

		// Make sure the cloud cluster take requests
		url := fmt.Sprintf("https://%s/apis/core.token-vendor/v1/public-key.read", domain)
		if err := waitForService(httpClient, url, numServiceRetries); err != nil {
			log.Fatalf("Failed to connect to the cloud cluster: %s. Please retry in 5 minutes.", err)
		}

		if err := setup.CreateAndPublishCredentialsToCloud(httpClient, auth); err != nil {
			log.Fatal(err)
		}
		if err := storeInK8sSecret(k8sLocalClientSet, baseNamespace, auth); err != nil {
			log.Fatal(fmt.Errorf("Failed to write auth secret: %v", err))
		}
		if err := gcr.UpdateGcrCredentials(k8sLocalClientSet, auth); err != nil {
			log.Fatal(err)
		}
	}

	// ensure tls certs
	whCert, whKey, err := ensureWebhookCerts(k8sLocalClientSet, baseNamespace)
	if err != nil {
		log.Fatalf("Failed to create tls certs for webhook: %v.", err)
	}

	log.Println("Initializing Synk")
	output, err := exec.Command(synkPath, "init").CombinedOutput()
	if err != nil {
		log.Fatalf("Synk init failed: %v. Synk output:\n%s\n", err, output)
	}

	appManagement := configutil.GetBoolean(vars, "APP_MANAGEMENT", true)
	// Use "robot" as a suffix for consistency for Synk deployments.
	installChartOrDie(k8sLocalClientSet, domain, registry, "base-robot", baseNamespace,
		"base-robot-0.0.1.tgz", whCert, whKey, appManagement)
	log.Println("Setup complete")
}

// create tls certs for the webhook if they don't exist or need an update
func ensureWebhookCerts(cs *kubernetes.Clientset, namespace string) (string, string, error) {
	sa, err := cs.CoreV1().Secrets(namespace).Get("robot-master-tls", metav1.GetOptions{})
	if err == nil && sa.Labels["cert-format"] == "v2" {
		// If we already have it and it has the right label, return the certs
		log.Print("Returning existing certificate.")
		return b64.URLEncoding.EncodeToString(sa.Data["tls.crt"]), b64.URLEncoding.EncodeToString(sa.Data["tls.key"]), nil
	}

	// Generate new certs
	// based on https://golang.org/src/crypto/tls/generate_cert.go

	priv, err := rsa.GenerateKey(rand.Reader, 2048)
	if err != nil {
		return "", "", errors.Wrap(err, "Failed to generate private key")
	}
	// ECDSA, ED25519 and RSA subject keys should have the DigitalSignature
	// KeyUsage bits set in the x509.Certificate template
	// Only RSA subject keys should have the KeyEncipherment KeyUsage bits set. In
	// the context of TLS this KeyUsage is particular to RSA key exchange and
	// authentication.
	keyUsage := x509.KeyUsageDigitalSignature | x509.KeyUsageKeyEncipherment

	serialNumberLimit := new(big.Int).Lsh(big.NewInt(1), 128)
	serialNumber, err := rand.Int(rand.Reader, serialNumberLimit)
	if err != nil {
		return "", "", errors.Wrap(err, "Failed to generate serial number")
	}

	template := x509.Certificate{
		SerialNumber: serialNumber,
		Subject: pkix.Name{
			Organization: []string{"robot-master." + namespace + ".svc"},
		},
		NotBefore: time.Now(),
		NotAfter:  time.Now().AddDate(100, 0, 0), // 100 years

		KeyUsage:              keyUsage,
		ExtKeyUsage:           []x509.ExtKeyUsage{x509.ExtKeyUsageServerAuth},
		BasicConstraintsValid: true,
		DNSNames:              []string{"robot-master." + namespace + ".svc"},
	}

	derBytes, err := x509.CreateCertificate(rand.Reader, &template, &template, &priv.PublicKey, priv)
	if err != nil {
		return "", "", errors.Wrap(err, "Failed to create certificate")
	}

	var crt bytes.Buffer
	if err := pem.Encode(&crt, &pem.Block{Type: "CERTIFICATE", Bytes: derBytes}); err != nil {
		return "", "", errors.Wrap(err, "Failed to write cert data")
	}

	var key bytes.Buffer
	privBytes, err := x509.MarshalPKCS8PrivateKey(priv)
	if err != nil {
		return "", "", errors.Wrap(err, "Unable to marshal private key")
	}
	if err := pem.Encode(&key, &pem.Block{Type: "PRIVATE KEY", Bytes: privBytes}); err != nil {
		return "", "", errors.Wrap(err, "Failed to write key data")
	}

	return b64.URLEncoding.EncodeToString(crt.Bytes()), b64.URLEncoding.EncodeToString(key.Bytes()), nil
}

func helmValuesStringFromMap(varMap map[string]string) string {
	varList := []string{}
	for k, v := range varMap {
		varList = append(varList, fmt.Sprintf("%s=%s", k, v))
	}
	return strings.Join(varList, ",")
}

// installChartOrDie installs a chart using Synk.
func installChartOrDie(cs *kubernetes.Clientset, domain, registry, name, namespace, chartPath, whCert, whKey string, appManagement bool) {
	// ensure namespace for chart exists
	if _, err := cs.CoreV1().Namespaces().Create(&corev1.Namespace{
		ObjectMeta: metav1.ObjectMeta{
			Name: namespace,
		},
	}); err != nil && !apierrors.IsAlreadyExists(err) {
		log.Fatalf("Failed to create %s namespace: %v.", namespace, err)
	}

	vars := helmValuesStringFromMap(map[string]string{
		"domain":               domain,
		"registry":             registry,
		"project":              *project,
		"app_management":       strconv.FormatBool(appManagement),
		"cr_syncer":            strconv.FormatBool(*crSyncer),
		"fluentd":              strconv.FormatBool(*fluentd),
		"docker_data_root":     *dockerDataRoot,
		"robot_authentication": strconv.FormatBool(*robotAuthentication),
		"robot.name":           *robotName,
		"webhook.tls.crt":      whCert,
		"webhook.tls.key":      whKey,
	})
	log.Printf("Installing %s chart using Synk from %s", name, chartPath)

	output, err := exec.Command(
		helmPath,
		"template",
		"--set-string", vars,
		"--name", name,
		"--namespace", namespace,
		filepath.Join(filesDir, chartPath),
	).CombinedOutput()
	if err != nil {
		log.Fatalf("Synk install of %s failed: %v\nHelm output:\n%s\n", name, err, output)
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
		log.Fatalf("Synk install of %s failed: %v\nSynk output:\n%s\n", name, err, output)
	}
}

// megeMaps returns `base` with `additions` added on top.
// I.e., if the same key is present in both maps, the one from `additions` wins.
func mergeMaps(base, additions map[string]string) map[string]string {
	result := make(map[string]string)
	for k, v := range base {
		result[k] = v
	}
	for k, v := range additions {
		result[k] = v
	}
	return result
}

func createOrUpdateRobot(k8sDynamicClient dynamic.Interface, labels map[string]string, annotations map[string]string) error {
	labels["cloudrobotics.com/robot-name"] = *robotName
	host := os.Getenv("HOST_HOSTNAME")
	if host != "" && labels["cloudrobotics.com/master-host"] == "" {
		labels["cloudrobotics.com/master-host"] = host
	}
	crc_version := os.Getenv("CRC_VERSION")
	if crc_version != "" {
		annotations["cloudrobotics.com/crc-version"] = crc_version
	}

	robotClient := k8sDynamicClient.Resource(robotGVR).Namespace("default")
	robot, err := robotClient.Get(*robotName, metav1.GetOptions{})
	if err != nil {
		if s, ok := err.(*apierrors.StatusError); ok && s.ErrStatus.Reason == metav1.StatusReasonNotFound {
			robot := &unstructured.Unstructured{}
			robot.SetKind("Robot")
			robot.SetAPIVersion("registry.cloudrobotics.com/v1alpha1")
			robot.SetName(*robotName)

			robot.SetLabels(labels)
			robot.SetAnnotations(annotations)
			robot.Object["spec"] = map[string]interface{}{
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

	// A robot with the same name already exists.
	robot.SetLabels(mergeMaps(robot.GetLabels(), labels))
	robot.SetAnnotations(mergeMaps(robot.GetAnnotations(), annotations))
	spec, ok := robot.Object["spec"].(map[string]interface{})
	if !ok {
		return fmt.Errorf("unmarshaling robot failed: spec is not a map")
	}
	spec["type"] = *robotType
	spec["project"] = *project
	_, err = robotClient.Update(robot, metav1.UpdateOptions{})
	return err
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
