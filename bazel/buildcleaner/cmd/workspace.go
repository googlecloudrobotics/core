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

// Util functions for interacting with the local workspace.
package cmd

import (
	"errors"
	"fmt"
	"os"
	"path/filepath"

	"github.com/bazelbuild/buildtools/wspace"
)

// checkWorkspacePath ensures that --workspace points to a workspace root.
func checkWorkspacePath() error {
	workspace, _ = wspace.FindWorkspaceRoot(workspace)
	if _, err := os.Stat(filepath.Join(workspace, "WORKSPACE")); err != nil {
		return errors.New("--workspace must specify the path to a Bazel workspace")
	}
	return nil
}

// getRepoPath locates the checkout of a given repository.
func getRepoPath(repo string) (string, error) {
	bazelOut, err := filepath.EvalSymlinks(filepath.Join(workspace, "bazel-out"))
	if err != nil {
		return "", err
	}
	path := filepath.Join(bazelOut, "..", "..", "..", "external", repo)
	if _, err := os.Stat(path); err != nil {
		return "", fmt.Errorf("failed to stat %v: %v", path, err)
	}
	return path, nil
}
