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

// Package synk contains functionality to synchronize a batch of resources
// with a cluster while correctly handling CRDs and deletions.
package synk

import (
	"context"
	"errors"
	"fmt"
	"regexp"
	"sort"
	"strconv"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/client-go/discovery"
	"k8s.io/client-go/kubernetes/scheme"
	"k8s.io/client-go/rest"
	"sigs.k8s.io/controller-runtime/pkg/client"
)

// Synk allows to synchronize sets of resources with a fixed cluster.
type Synk struct {
	restcfg   *rest.Config
	client    client.Client
	discovery discovery.DiscoveryInterface
}

// New returns a new Synk object that acts against the cluster for the given configuration.
func New(cfg *rest.Config) (*Synk, error) {
	c, err := newClient(cfg)
	if err != nil {
		return nil, err
	}
	d, err := discovery.NewDiscoveryClientForConfig(cfg)
	if err != nil {
		return nil, err
	}
	return &Synk{
		restcfg:   cfg,
		client:    c,
		discovery: d,
	}, nil
}

func newClient(cfg *rest.Config) (client.Client, error) {
	sc := runtime.NewScheme()
	scheme.AddToScheme(sc)
	apps.AddToScheme(sc)
	return client.New(cfg, client.Options{Scheme: sc})
}

// TODO: determine options that allow us to be semantically compatible with
// vanilla kubectl apply.
type ApplyOptions struct {
	name    string
	version int32
}

func (k *Synk) Apply(
	ctx context.Context,
	name string,
	opts *ApplyOptions,
	resources ...*unstructured.Unstructured,
) (*apps.ResourceSet, error) {
	if opts == nil {
		opts = &ApplyOptions{}
	}
	opts.name = name

	for _, r := range resources {
		fmt.Println(resourceKey(r))
	}

	return nil, errors.New("not implemented")
}

func resourceSetName(s string, v int32) string {
	return fmt.Sprintf("%s.v%d", s, v)
}

var namePat = regexp.MustCompile(`^([a-z0-9]+)\.v([0-9]+)$`)

func decodeResourceSetName(s string) (string, int32, bool) {
	res := namePat.FindStringSubmatch(s)
	if len(res) == 0 {
		return "", 0, false
	}
	version, err := strconv.Atoi(res[2])
	if err != nil {
		panic(err)
	}
	return res[1], int32(version), true
}

func sortResources(res []*unstructured.Unstructured) {
	sort.Slice(res, func(i, j int) bool {
		if res[i].GetAPIVersion() > res[j].GetAPIVersion() {
			return false
		}
		if res[i].GetKind() > res[j].GetKind() {
			return false
		}
		if res[i].GetNamespace() > res[j].GetNamespace() {
			return false
		}
		return res[i].GetName() < res[j].GetName()
	})
}

func resourceKey(r *unstructured.Unstructured) string {
	return fmt.Sprintf("%s/%s/%s/%s",
		r.GetAPIVersion(),
		r.GetKind(),
		r.GetNamespace(),
		r.GetName(),
	)
}
