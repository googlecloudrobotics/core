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

// render-chartassignment renders ChartAssignments offline using the same code
// path as the chart-assignment-controller and writes the resulting manifests
// to disk. It is intended for diffing the output of different controller
// versions.
//
// Usage:
//
//	render-chartassignment -out-dir DIR FILE...
//
// Each input file must contain a single ChartAssignment. For each one, a
// <name>.yaml file is written to DIR containing the rendered resources, sorted
// by apiVersion, kind, namespace and name. If rendering fails, a <name>.error
// file containing the error is written instead.
package main

import (
	"bytes"
	"flag"
	"fmt"
	"os"
	"path/filepath"
	"sort"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"github.com/googlecloudrobotics/core/src/go/pkg/controller/chartassignment"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"sigs.k8s.io/yaml"
)

var outDir = flag.String("out-dir", "", "Directory to write rendered manifests to.")

func main() {
	flag.Usage = func() {
		fmt.Fprintf(flag.CommandLine.Output(), "Usage: %s -out-dir DIR FILE...\n", os.Args[0])
		flag.PrintDefaults()
	}
	flag.Parse()
	if *outDir == "" || flag.NArg() == 0 {
		flag.Usage()
		os.Exit(2)
	}
	if err := os.MkdirAll(*outDir, 0o755); err != nil {
		fmt.Fprintf(os.Stderr, "create out dir: %v\n", err)
		os.Exit(1)
	}

	var ok, failed int
	for _, f := range flag.Args() {
		name, err := renderFile(f)
		if err != nil {
			// Errors about the tool itself (e.g. unreadable input) are fatal,
			// render errors are written to the output directory.
			fmt.Fprintf(os.Stderr, "%s: %v\n", f, err)
			os.Exit(1)
		}
		if name == "" {
			ok++
		} else {
			failed++
			fmt.Fprintf(os.Stderr, "%s: render failed, see %s\n", f, name)
		}
	}
	fmt.Printf("rendered %d chartassignments to %s (%d ok, %d failed)\n", ok+failed, *outDir, ok, failed)
}

// renderFile renders a single ChartAssignment file. It returns the path of the
// error file if rendering failed.
func renderFile(path string) (string, error) {
	b, err := os.ReadFile(path)
	if err != nil {
		return "", err
	}
	var as apps.ChartAssignment
	if err := yaml.Unmarshal(b, &as); err != nil {
		return "", fmt.Errorf("decode ChartAssignment: %w", err)
	}
	if as.Kind != "ChartAssignment" || as.Name == "" {
		return "", fmt.Errorf("not a ChartAssignment with a name (kind=%q, name=%q)", as.Kind, as.Name)
	}

	resources, err := chartassignment.Render(&as)
	if err != nil {
		errFile := filepath.Join(*outDir, as.Name+".error")
		if werr := os.WriteFile(errFile, []byte(err.Error()+"\n"), 0o644); werr != nil {
			return "", werr
		}
		return errFile, nil
	}

	out, err := encode(resources)
	if err != nil {
		return "", err
	}
	return "", os.WriteFile(filepath.Join(*outDir, as.Name+".yaml"), out, 0o644)
}

func sortKey(u *unstructured.Unstructured) string {
	return u.GetAPIVersion() + "/" + u.GetKind() + "/" + u.GetNamespace() + "/" + u.GetName()
}

// encode serializes the resources in a stable order so that outputs of
// different versions can be diffed.
func encode(resources []*unstructured.Unstructured) ([]byte, error) {
	sort.SliceStable(resources, func(i, j int) bool {
		return sortKey(resources[i]) < sortKey(resources[j])
	})
	var buf bytes.Buffer
	for _, r := range resources {
		// sigs.k8s.io/yaml sorts map keys, which keeps the output deterministic.
		b, err := yaml.Marshal(r.Object)
		if err != nil {
			return nil, fmt.Errorf("encode %s: %w", sortKey(r), err)
		}
		fmt.Fprintf(&buf, "---\n# %s\n", sortKey(r))
		buf.Write(b)
	}
	return buf.Bytes(), nil
}
