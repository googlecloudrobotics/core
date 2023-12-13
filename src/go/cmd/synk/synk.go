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
	"log/slog"
	"os"
	"strings"
	"time"

	"github.com/cenkalti/backoff"
	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"github.com/googlecloudrobotics/core/src/go/pkg/synk"
	"github.com/googlecloudrobotics/ilog"
	"github.com/pkg/errors"
	"github.com/spf13/cobra"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"k8s.io/cli-runtime/pkg/genericclioptions"
	"k8s.io/cli-runtime/pkg/resource"
	"k8s.io/client-go/dynamic"
	_ "k8s.io/client-go/plugin/pkg/client/auth"
)

const (
	retryBackoff = 5 * time.Second
)

var (
	maxQPS  int
	retries uint64

	cmdRoot = &cobra.Command{
		Use:   "synk",
		Short: "A tool to sync manifests with a cluster.",
	}
	cmdInit = &cobra.Command{
		Use:   "init",
		Short: "Initialize cluster for use with synk.",
		Run:   runInit,
	}
	cmdApply = &cobra.Command{
		Use:   "apply",
		Short: "Apply manifests to the cluster.",
		Run:   runApply,
	}
	cmdDelete = &cobra.Command{
		Use:   "delete",
		Short: "Delete all ResourceSets for the name.",
		Run:   runDelete,
	}

	restOpts     = genericclioptions.NewConfigFlags(true)
	resourceOpts = genericclioptions.NewResourceBuilderFlags()
)

func main() {
	logHandler := ilog.NewLogHandler(slog.LevelInfo, os.Stderr)
	slog.SetDefault(slog.New(logHandler))

	restOpts.AddFlags(cmdRoot.PersistentFlags())
	resourceOpts.AddFlags(cmdApply.PersistentFlags())

	cmdRoot.PersistentFlags().IntVar(&maxQPS, "max-qps", 50, "max number of calls to the apiserver per second")
	cmdApply.PersistentFlags().Uint64Var(&retries, "retries", 60, "max number of retries for transient errors, with a 5 second constant backoff")

	cmdRoot.AddCommand(cmdInit)
	cmdRoot.AddCommand(cmdApply)
	cmdRoot.AddCommand(cmdDelete)

	if err := cmdRoot.Execute(); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}

func newSynk() (*synk.Synk, error) {
	restcfg, err := restOpts.ToRESTConfig()
	if err != nil {
		return nil, errors.Wrap(err, "get config")
	}
	restcfg.QPS = float32(maxQPS)
	restcfg.Burst = maxQPS * 2
	discovery, err := restOpts.ToDiscoveryClient()
	if err != nil {
		return nil, errors.Wrap(err, "get discovery client")
	}
	client, err := dynamic.NewForConfig(restcfg)
	if err != nil {
		return nil, errors.Wrap(err, "create dynamic client")
	}
	s := synk.New(client, discovery)

	// Invalidate to be safe. It seems that a persistent discovery cache
	// likes to stay out of sync way too often.
	discovery.Invalidate()

	return s, nil
}

func runInit(cmd *cobra.Command, args []string) {
	s, err := newSynk()
	if err != nil {
		fmt.Fprintln(os.Stderr, err.Error())
		os.Exit(1)
	}
	if err := s.Init(); err != nil {
		fmt.Fprintln(os.Stderr, err.Error())
		os.Exit(1)
	}
	fmt.Fprintln(os.Stderr, "Initialized successfully")
}

func runDelete(cmd *cobra.Command, args []string) {
	if len(args) != 1 {
		fmt.Fprintln(os.Stderr, "unrecognized number of arguments, exactly one (name) expected")
		os.Exit(2)
	}
	s, err := newSynk()
	if err != nil {
		fmt.Fprintln(os.Stderr, err.Error())
		os.Exit(1)
	}
	if err := s.Delete(context.Background(), args[0]); err != nil {
		fmt.Fprintln(os.Stderr, err.Error())
		os.Exit(1)
	}
	fmt.Fprintln(os.Stderr, "Deleted successfully")
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
	// If a target namesapce for the chart is given, enforce it.
	namespace, enforceNamespace, err := restOpts.ToRawKubeConfigLoader().Namespace()
	if err != nil {
		return err
	}
	filenameOpts := resourceOpts.FileNameFlags.ToOptions()

	result := resource.NewBuilder(restOpts).
		ContinueOnError().
		Unstructured(). // Must be at the top.
		Local().
		FilenameParam(false, &filenameOpts).
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

	s, err := newSynk()
	if err != nil {
		return err
	}
	opts := &synk.ApplyOptions{
		Namespace:        namespace,
		EnforceNamespace: enforceNamespace,
		Log:              logAction,
	}
	if err := backoff.Retry(
		func() error {
			_, err := s.Apply(context.Background(), name, opts, resources...)
			if err != nil {
				if synk.IsTransientErr(err) {
					return err
				}
				return backoff.Permanent(err)
			}
			return nil
		},
		backoff.WithMaxRetries(backoff.NewConstantBackOff(retryBackoff), retries),
	); err != nil {
		return errors.Wrap(err, "apply files")
	}
	return nil
}

func logAction(r *unstructured.Unstructured, action apps.ResourceAction, status, msg string) {
	// Remove some visual clutter by only showing the resource for successes.
	if status == synk.StatusSuccess {
		fmt.Fprintf(os.Stderr, "[%s] %s %s/%s %s/%s\n",
			strings.ToUpper(status), action,
			r.GetAPIVersion(), r.GetKind(),
			r.GetNamespace(), r.GetName(),
		)
		return
	}
	fmt.Fprintf(os.Stderr, "[%s] %s %s/%s %s/%s: %s\n",
		strings.ToUpper(status), action,
		r.GetAPIVersion(), r.GetKind(),
		r.GetNamespace(), r.GetName(),
		msg)
}
