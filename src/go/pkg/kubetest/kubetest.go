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

// Package kubetest provides functionality to create local Kubernetes test
// clusters and run tests against them.
package kubetest

import (
	"bytes"
	"context"
	"fmt"
	"html/template"
	"io/ioutil"
	"log"
	"os"
	"os/exec"
	"reflect"
	"runtime"
	"strings"
	"testing"
	"time"

	"github.com/cenkalti/backoff"
	"github.com/googlecloudrobotics/core/src/go/pkg/gcr"
	"github.com/pkg/errors"
	"golang.org/x/sync/errgroup"
	apps "k8s.io/api/apps/v1"
	core "k8s.io/api/core/v1"
	rbac "k8s.io/api/rbac/v1"
	k8serrors "k8s.io/apimachinery/pkg/api/errors"
	meta "k8s.io/apimachinery/pkg/apis/meta/v1"
	k8sruntime "k8s.io/apimachinery/pkg/runtime"
	"k8s.io/client-go/kubernetes/scheme"
	"k8s.io/client-go/rest"
	"k8s.io/client-go/tools/clientcmd"
	"sigs.k8s.io/controller-runtime/pkg/client"
	kindcluster "sigs.k8s.io/kind/pkg/cluster"
	kindconfig "sigs.k8s.io/kind/pkg/cluster/config"
	kinddefaults "sigs.k8s.io/kind/pkg/cluster/config/defaults"
	"sigs.k8s.io/yaml"
)

// Environment encapsulates a set of clusters and can run tests against them.
type Environment struct {
	t           *testing.T
	cfg         Config
	scheme      *k8sruntime.Scheme
	helmPath    string
	synkPath    string
	clusters    map[string]*cluster
	uniqCounter int
}

type Config struct {
	// The clusters that are provisioned for the test environment.
	Clusters []ClusterConfig

	// Registration function for additional resource types to a scheme.
	SchemeFunc func(*k8sruntime.Scheme) error
}

type ClusterConfig struct {
	Name string
}

type cluster struct {
	genName string
	cfg     ClusterConfig
	ctx     *kindcluster.Context
	restCfg *rest.Config
}

// New creates a new test environment.
func New(t *testing.T, cfg Config) *Environment {
	e := &Environment{
		helmPath: "../kubernetes_helm/helm",
		synkPath: "src/go/cmd/synk/linux_amd64_stripped/synk",
		t:        t,
		cfg:      cfg,
		scheme:   k8sruntime.NewScheme(),
		clusters: map[string]*cluster{},
	}
	if cfg.SchemeFunc != nil {
		cfg.SchemeFunc(e.scheme)
	}
	scheme.AddToScheme(e.scheme)

	var g errgroup.Group
	// Setup cluster concurrently.
	for _, cfg := range cfg.Clusters {
		// Make name unique to avoid collisions across parallel tests.
		uniqName := fmt.Sprintf("%s-%x", cfg.Name, time.Now().UnixNano())
		t.Logf("Assigned unique name %q to cluster %q", uniqName, cfg.Name)

		cluster := &cluster{
			genName: uniqName,
			cfg:     cfg,
		}
		e.clusters[cfg.Name] = cluster

		g.Go(func() error {
			if err := setupCluster(e.synkPath, cluster); err != nil {
				// If cluster has already been created, delete it.
				if cluster.ctx != nil && os.Getenv("NO_TEARDOWN") == "" {
					cluster.ctx.Delete()
				}
				return errors.Wrapf(err, "Create cluster %q", cfg.Name)
			}
			log.Printf("Created cluster %q", cfg.Name)
			return nil
		})
	}
	if err := g.Wait(); err != nil {
		t.Fatal(err)
	}
	return e
}

func (e *Environment) Ctx() context.Context {
	return context.Background()
}

func helmValues(vars map[string]string) string {
	list := []string{}
	for k, v := range vars {
		list = append(list, fmt.Sprintf("%s=%s", k, v))
	}
	return strings.Join(list, ",")
}

// InstallChartArchive installs a Helm chart from a tarball on disk into a cluster.
// Arguments are provided as a map where the keys are JSON paths.
func (e *Environment) InstallChartArchive(cluster, name, namespace, path string, args map[string]string) {
	c, ok := e.clusters[cluster]
	if !ok {
		e.t.Fatalf("Unknown cluster %q", cluster)
	}

	output, err := exec.Command(
		e.helmPath,
		"template",
		"--set-string", helmValues(args),
		"--name", name,
		path,
	).CombinedOutput()
	if err != nil {
		e.t.Fatalf("Synk install of %s failed: %v\nHelm output:\n%s\n", name, err, output)
	}
	cmd := exec.Command(
		e.synkPath,
		"apply",
		name,
		"--kubeconfig", c.ctx.KubeConfigPath(),
		"-n", namespace,
		"-f", "-",
	)
	// Helm writes the templated manifests and errors alike to stderr.
	// So we can just take the combined output as is.
	cmd.Stdin = bytes.NewReader(output)

	if output, err = cmd.CombinedOutput(); err != nil {
		e.t.Fatalf("Synk install of %s failed: %v\nSynk output:\n%s\n", name, err, output)
	}
}

// Client returns a new client for the cluster.
func (e *Environment) Client(cluster string) client.Client {
	c, ok := e.clusters[cluster]
	if !ok {
		e.t.Fatalf("cluster with name %q does not exist", cluster)
	}
	client, err := client.New(c.restCfg, client.Options{
		Scheme: e.scheme,
	})
	if err != nil {
		e.t.Fatalf("Create client for cluster %q: %s", cluster, err)
	}
	return client
}

// Teardown destroys all clusters that were created for the environment.
func (e *Environment) Teardown() {
	if os.Getenv("NO_TEARDOWN") != "" {
		log.Printf("Skipping teardown")
		return
	}
	log.Println("Tearing down...")

	for name, c := range e.clusters {
		if err := c.ctx.Delete(); err != nil {
			e.t.Errorf("Delete cluster %q (%q): %s", name, c.genName, err)
		} else {
			log.Printf("Deleted cluster %q (%q)", name, c.genName)
		}
	}
}

type TestFunc func(*testing.T, *Fixture)

// Run takes a list of TestFuncs and executes them as subtests.
func (e *Environment) Run(tests ...TestFunc) {
	for _, test := range tests {
		f := e.New(test)
		e.t.Run(f.name, func(t *testing.T) {
			// Recover from panics to ensure that defer Teardown will run.
			defer func() {
				if err := recover(); err != nil {
					t.Errorf("panic: %s", err)
				}
			}()
			f.testFn(t, f)
		})
	}
}

// Uniq takes a string and makes it unique. It should be used to generate collision-free
// names for namespaces or cluster-wide resources from subtests.
func (e *Environment) Uniq(s string) string {
	e.uniqCounter++
	return fmt.Sprintf("%s-%d", s, e.uniqCounter)
}

// Fixture provides functionality for a single test that is run against an environment.
type Fixture struct {
	t      *testing.T
	name   string
	env    *Environment
	testFn TestFunc
}

// New creates a new Fixture for a test function.
func (env *Environment) New(testFn TestFunc) *Fixture {
	return &Fixture{
		name:   runtime.FuncForPC(reflect.ValueOf(testFn).Pointer()).Name(),
		t:      env.t,
		testFn: testFn,
		env:    env,
	}
}

func (f *Fixture) Ctx() context.Context {
	return context.Background()
}

// ObjectKey extracts a namespace/name key from the given object.
func (f *Fixture) ObjectKey(o k8sruntime.Object) client.ObjectKey {
	k, err := client.ObjectKeyFromObject(o)
	if err != nil {
		f.t.Fatalf("get key from object: %s", err)
	}
	return k
}

// Uniq takes a string and makes it unique. It should be used to generate collision-free
// names for namespaces or cluster-wide resources from subtests.
func (f *Fixture) Uniq(s string) string {
	return f.env.Uniq(s)
}

// FromYAML expands a YAML template with the given vals and unmarshals it into dst.
// dst is typically of type *unstructured.Unstructured or a fully specified type
// for a Kubernetes resource.
func (f *Fixture) FromYAML(tmpl string, vals, dst interface{}) {
	f.t.Helper()

	t, err := template.New("").Parse(tmpl)
	if err != nil {
		f.t.Fatalf("Invalid template: %s", err)
	}
	var buf bytes.Buffer
	if err := t.Execute(&buf, vals); err != nil {
		f.t.Fatalf("Execute template: %s", err)
	}
	if err := yaml.Unmarshal(bytes.TrimSpace(buf.Bytes()), dst); err != nil {
		f.t.Fatal(err)
	}
}

// Client returns a new client for the cluster.
func (f *Fixture) Client(cluster string) client.Client {
	f.t.Helper()
	return f.env.Client(cluster)
}

// setupCluster creates a kind cluster and installs synk if necessary.
func setupCluster(synkPath string, cluster *cluster) error {
	kindcfg := &kindconfig.Cluster{
		Nodes: []kindconfig.Node{
			{
				Role:  kindconfig.ControlPlaneRole,
				Image: kinddefaults.Image,
			}, {
				Role:  kindconfig.WorkerRole,
				Image: kinddefaults.Image,
			},
		},
	}
	cluster.ctx = kindcluster.NewContext(cluster.genName)

	if err := cluster.ctx.Create(kindcfg); err != nil {
		return errors.Wrapf(err, "create cluster %q", cluster.genName)
	}
	kubecfgRaw, err := ioutil.ReadFile(cluster.ctx.KubeConfigPath())
	if err != nil {
		return errors.Wrap(err, "read kube config")
	}
	kubecfg, err := clientcmd.NewClientConfigFromBytes(kubecfgRaw)
	if err != nil {
		return errors.Wrap(err, "decode kube config")
	}
	cluster.restCfg, err = kubecfg.ClientConfig()
	if err != nil {
		return errors.Wrap(err, "get rest config")
	}

	// Setup permissive binding we also have in cloud and robot clusters.
	ctx := context.Background()

	c, err := client.New(cluster.restCfg, client.Options{})
	if err != nil {
		return errors.Wrap(err, "create client")
	}
	if err := c.Create(ctx, &rbac.ClusterRoleBinding{
		ObjectMeta: meta.ObjectMeta{
			Name: "permissive-binding",
		},
		RoleRef: rbac.RoleRef{
			APIGroup: "rbac.authorization.k8s.io",
			Kind:     "ClusterRole",
			Name:     "cluster-admin",
		},
		Subjects: []rbac.Subject{{
			APIGroup: "rbac.authorization.k8s.io",
			Kind:     "Group",
			Name:     "system:serviceaccounts",
		}},
	}); err != nil {
		return errors.Wrap(err, "create permissive role binding")
	}

	// Setup service account and create image pull secrets.
	if token := os.Getenv("ACCESS_TOKEN"); token != "" {
		// Use the same secret name as the GCR credential refresher would
		// on robots.
		// This makes some testing of components easier, that assume this
		// secret to exist, e.g. ChartAssignment controller.
		secret := &core.Secret{
			ObjectMeta: meta.ObjectMeta{
				Namespace: "default",
				Name:      gcr.SecretName,
			},
			Type: core.SecretTypeDockercfg,
			Data: map[string][]byte{
				".dockercfg": gcr.DockerCfgJSON(token),
			},
		}
		if err := c.Create(ctx, secret); err != nil {
			return errors.Wrap(err, "create pull secret")
		}
		if err := backoff.Retry(
			func() error {
				var sa core.ServiceAccount
				err := c.Get(ctx, client.ObjectKey{"default", "default"}, &sa)
				if k8serrors.IsNotFound(err) {
					return errors.New("not found")
				} else if err != nil {
					return backoff.Permanent(errors.Wrap(err, "get service account"))
				}
				sa.ImagePullSecrets = append(sa.ImagePullSecrets, core.LocalObjectReference{
					Name: gcr.SecretName,
				})
				if err = c.Update(ctx, &sa); k8serrors.IsConflict(err) {
					return fmt.Errorf("conflict")
				} else if err != nil {
					return backoff.Permanent(errors.Wrap(err, "update service account"))
				}
				return nil
			},
			backoff.WithMaxRetries(backoff.NewConstantBackOff(time.Second), 60),
		); err != nil {
			return errors.Wrap(err, "inject pull secret")
		}
	}

	// Install Tiller. We wait for all node taints to be removed (e.g. NotReady)
	// so Tiller doesn't fail permanently (see b/128660997).
	if err := backoff.Retry(
		func() error {
			var nds core.NodeList
			if err := c.List(ctx, &nds); err != nil {
				return backoff.Permanent(err)
			}
			for _, n := range nds.Items {
				if len(n.Spec.Taints) == 0 {
					return nil
				}
			}
			return fmt.Errorf("taints not removed")
		},
		backoff.WithMaxRetries(backoff.NewConstantBackOff(time.Second), 120),
	); err != nil {
		return errors.Wrap(err, "wait for node taints to be removed")
	}
	cmd := exec.Command(
		synkPath,
		"init",
		"--kubeconfig", cluster.ctx.KubeConfigPath(),
	)
	if output, err := cmd.CombinedOutput(); err != nil {
		return errors.Errorf("install Helm: %v; output:\n%s\n", err, output)
	}
	return nil
}

// DeploymentReady returns a condition func that checks whether all replicas of a deployment
// are available.
func DeploymentReady(ctx context.Context, c client.Client, namespace, name string) error {
	var d apps.Deployment
	if err := c.Get(ctx, client.ObjectKey{namespace, name}, &d); err != nil {
		return backoff.Permanent(errors.Wrapf(err, "get deployment %s/%s", namespace, name))
	}
	if d.Spec.Replicas == nil {
		if d.Status.ReadyReplicas <= 0 {
			return fmt.Errorf("Replicas not ready")
		}
		return nil
	} else if d.Status.ReadyReplicas != *d.Spec.Replicas {
		return fmt.Errorf("Replicas not ready")
	}
	return nil
}
