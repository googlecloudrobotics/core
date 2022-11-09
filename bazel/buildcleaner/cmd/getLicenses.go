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

package cmd

import (
	"bufio"
	"encoding/xml"
	"fmt"
	"io"
	"io/ioutil"
	"os"
	"path/filepath"
	"sort"
	"strings"

	"github.com/bazelbuild/buildtools/build"
	license "github.com/ryanuber/go-license"
	"github.com/spf13/cobra"
)

// isMirror returns true if the URL looks like a mirror rather than the
// original source of the archive.
func isMirror(url string) bool {
	return strings.Contains(url, "mirror.bazel.build")
}

// getSourceURL determines the source URL of the archive by filtering out
// mirror URLs.
func getSourceURL(r *build.Rule) (string, error) {
	urls := getURLs(r)
	var sourceURL string
	for _, u := range urls {
		if isMirror(u) {
			continue
		}
		if sourceURL != "" {
			return "", fmt.Errorf("multiple source URLs for %v in %v", r.Name(), urls)
		}
		sourceURL = u
	}
	if sourceURL != "" {
		return sourceURL, nil
	}
	return "", fmt.Errorf("no source URLs for %v in %v", r.Name(), urls)
}

// getRepoID gets an identifier for the archive by transforming the source URL.
// Examples:
//
//	https://github.com/libexpat/libexpat/archive/R_2_2_4.tar.gz
//	-> github.com/libexpat/libexpat
//	http://www.dest-unreach.org/socat/download/socat-1.7.3.2.tar.gz
//	-> www.dest-unreach.org/socat/download
func getRepoID(u string) string {
	u = strings.TrimPrefix(u, "http://")
	u = strings.TrimPrefix(u, "https://")
	u = filepath.Dir(u)
	u = strings.TrimSuffix(u, "/archive")
	return u
}

// mapToString returns a sorted, spaced, comma-separated list of map keys.
func mapToString(m map[string]bool) string {
	var keys []string
	for k := range m {
		keys = append(keys, k)
	}
	sort.Strings(keys)
	return strings.Join(keys, ", ")
}

// charsetReader returns a reader for the given charset. This is required to
// read ASCII XML files.
func charsetReader(charset string, input io.Reader) (io.Reader, error) {
	switch strings.ToLower(charset) {
	case "ascii":
		return input, nil
	default:
		return nil, fmt.Errorf("can't decode XML document using charset %q", charset)
	}
}

// ROSPackage contains a subset of the data in a ROS package.xml file.
type ROSPackage struct {
	XMLName xml.Name `xml:"package"`
	License string   `xml:"license"`
}

// getBazelLicense tries to read license info from BUILD.bazel, by finding
// lines such as:
//
//	licenses(["notice"])  # BSD
func getBazelLicense(repoDir string) (string, error) {
	f, err := os.Open(filepath.Join(repoDir, "BUILD.bazel"))
	if err != nil {
		if os.IsNotExist(err) {
			return "", nil
		}
		return "", err
	}
	defer f.Close()
	scanner := bufio.NewScanner(f)
	for scanner.Scan() {
		l := scanner.Text()
		if !strings.HasPrefix(l, "licenses(") || !strings.Contains(l, "#") {
			continue
		}
		l = strings.SplitN(l, "#", 2)[1]
		return strings.TrimSpace(l), nil
	}

	return "", scanner.Err()
}

// getFileLicense searches recursively for license files inside the repo.
func getFileLicense(repoDir string) (string, error) {
	licenseMap := make(map[string]bool)
	err := searchForLicense(repoDir, licenseMap)
	if err != nil {
		return "", err
	}
	return mapToString(licenseMap), nil
}

// searchForLicense implements the recursive search in getFileLicense.
func searchForLicense(path string, licenseMap map[string]bool) error {
	l, err := license.NewFromDir(path)
	if err == nil {
		licenseMap[l.Type] = true
		return nil
	}
	// If we didn't find a license; recurse.
	dirents, err := ioutil.ReadDir(path)
	if err != nil {
		return err
	}
	for _, d := range dirents {
		if !d.IsDir() {
			continue
		}
		if err := searchForLicense(filepath.Join(path, d.Name()), licenseMap); err != nil {
			return err
		}
	}
	return nil
}

// getLicense wraps the different license finding methods.
func getLicense(repoDir string) (string, error) {
	// First check for hardcoded results for otherwise-difficult repos.
	l, err := getBazelLicense(repoDir)
	if err != nil {
		return "", err
	}
	if l != "" {
		return l, nil
	}

	l, err = getFileLicense(repoDir)
	if l != "" {
		return l, nil
	}
	return "???", nil
}

// printLicenses tries to determine and print the license for the given
// http_archive rule.
func printLicense(r *build.Rule) error {
	u, err := getSourceURL(r)
	if err != nil {
		return err
	}
	repoDir, err := getRepoPath(r.Name())
	if err != nil {
		return err
	}
	l, err := getLicense(repoDir)
	if err != nil {
		return err
	}
	fmt.Printf("%v\t%s\n", getRepoID(u), l)
	return nil
}

// printLicenses tries to determine and print the license for each http_archive
// in the repo.
func printLicenses(ws *build.File) {
	kinds := []string{
		"http_archive",
		"new_http_archive",
	}
	for _, kind := range kinds {
		for _, r := range ws.Rules(kind) {
			if err := printLicense(r); err != nil {
				fmt.Fprintf(os.Stderr, "ERROR: %v: %v\n", r.Name(), err)
			}
		}
	}
}

// getLicensesCmd represents the getLicenses command.
var getLicensesCmd = &cobra.Command{
	Use:   "getLicenses",
	Short: "Print license information for dependencies",
	Long: `getLicenses finds http_archive entries in the WORKSPACE and
tries to determine and print license information.`,
	RunE: func(cmd *cobra.Command, args []string) error {
		if err := checkWorkspacePath(); err != nil {
			return err
		}
		workspaceFile := filepath.Join(workspace, "WORKSPACE")
		ws, err := loadWorkspace(workspaceFile)
		if err != nil {
			return err
		}
		printLicenses(ws)
		return nil
	},
}

func init() {
	RootCmd.AddCommand(getLicensesCmd)
}
