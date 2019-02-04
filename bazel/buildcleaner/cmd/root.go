// Copyright 2019 The Google Cloud Robotics Authors
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

package cmd

import (
	"fmt"
	"os"

	"github.com/spf13/cobra"
)

var dryRun bool
var workspace string

var RootCmd = &cobra.Command{
	Use:   "buildcleaner",
	Short: "Apply automatic rewrites to BUILD and WORKSPACE files",
	Long: `buildcleaner applies automatic rewrites to BUILD and WORKSPACE
	files, such as adding mirrors for external repositories.`,
}

func Execute() {
	if err := RootCmd.Execute(); err != nil {
		fmt.Println(err)
		os.Exit(1)
	}
}

func init() {
	// These flags are global for the entire application.
	RootCmd.PersistentFlags().BoolVar(&dryRun, "dry_run", false, "print new rules without changing BUILD files")
	RootCmd.PersistentFlags().StringVar(&workspace, "workspace", "", "path to workspace")
}
