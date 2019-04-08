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
	"fmt"
	"os"

	"github.com/googlecloudrobotics/core/src/go/pkg/synk"
	"github.com/pkg/errors"
	"github.com/spf13/cobra"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"k8s.io/cli-runtime/pkg/genericclioptions"
	"k8s.io/cli-runtime/pkg/genericclioptions/resource"
	_ "k8s.io/client-go/plugin/pkg/client/auth"
)

var (
	cmdRoot = &cobra.Command{
		Use:   "synk",
		Short: "A tool to sync manifests with a cluster.",
	}
	cmdApply = &cobra.Command{
		Use:   "apply",
		Short: "Apply manifests to the cluster",
		Run:   runApply,
	}

	restOpts     = genericclioptions.NewConfigFlags()
	resourceOpts = genericclioptions.NewResourceBuilderFlags()
)

func main() {
	restOpts.AddFlags(cmdRoot.PersistentFlags())
	resourceOpts.AddFlags(cmdApply.PersistentFlags())

	cmdRoot.AddCommand(cmdApply)

	if err := cmdRoot.Execute(); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}

func runApply(cmd *cobra.Command, args []string) {
	if len(args) != 1 {
		fmt.Fprintln(os.Stderr, "unrecognized number of arguments, exactly one (name) expected")
		os.Exit(2)
	}
	if err := apply(args[0]); err != nil {
		fmt.Fprintln(os.Stderr, err.Error())
		os.Exit(1)
	}
}

func apply(name string) error {
	namespace, enforceNamespace, err := restOpts.ToRawKubeConfigLoader().Namespace()
	if err != nil {
		return err
	}
	filenameOpts := resourceOpts.FileNameFlags.ToOptions()

	result := resource.NewBuilder(restOpts).
		Unstructured().
		NamespaceParam(namespace).
		DefaultNamespace().
		FilenameParam(enforceNamespace, &filenameOpts).
		Flatten().
		Do()

	if result.Err() != nil {
		return errors.Wrap(result.Err(), "get files")
	}
	infos, err := result.Infos()
	if err != nil {
		return errors.Wrap(err, "get file information")
	}
	var resources []*unstructured.Unstructured
	for _, i := range infos {
		resources = append(resources, i.Object.(*unstructured.Unstructured))
	}

	restcfg, err := restOpts.ToRESTConfig()
	if err != nil {
		return errors.Wrap(err, "get config")
	}
	k, err := synk.New(restcfg)
	if err != nil {
		return errors.Wrap(err, "setup client")
	}
	opts := &synk.ApplyOptions{}
	if _, err := k.Apply(context.Background(), name, opts, resources...); err != nil {
		return errors.Wrap(err, "apply files")
	}
	return nil
}
