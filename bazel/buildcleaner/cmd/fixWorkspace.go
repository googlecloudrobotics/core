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
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"fmt"
	"io"
	"io/ioutil"
	"log"
	"net/http"
	"net/url"
	"path"
	"path/filepath"
	"strings"
	"sync"

	"github.com/bazelbuild/buildtools/build"
	"github.com/spf13/cobra"
)

// loadWorkspace parses the WORKSPACE file.
func loadWorkspace(workspaceFile string) (*build.File, error) {
	data, err := ioutil.ReadFile(workspaceFile)
	if err != nil {
		return nil, err
	}
	return build.Parse(workspaceFile, data)
}

// saveWorkspace formats and writes the WORKSPACE file.
func saveWorkspace(workspaceFile string, ws *build.File) error {
	build.Rewrite(ws)
	data := build.Format(ws)
	return ioutil.WriteFile(workspaceFile, data, 0644)
}

// committish returns the commit or tag of the repo.
func committish(r *build.Rule) string {
	commit := r.AttrString("commit")
	if commit != "" {
		return commit
	}
	return r.AttrString("tag")
}

// archiveURLs calculates the URL(s) of the git repo archive.
func archiveURLs(r *build.Rule) ([]string, error) {
	remote := r.AttrString("remote")
	if !strings.HasPrefix(remote, "https://github.com") {
		return nil, errors.New("not a github repo")
	}
	repoURL := strings.TrimSuffix(remote, ".git")
	archiveURL := fmt.Sprintf("%s/archive/%s.tar.gz", repoURL, committish(r))
	return []string{archiveURL}, nil
}

// archivePrefix calculates the prefix to strip from files inside a git repo
// archive.
func archivePrefix(r *build.Rule) string {
	// convert v3.0.0 to 3.0.0
	version := strings.TrimPrefix(committish(r), "v")
	repoURL := strings.TrimSuffix(r.AttrString("remote"), ".git")
	repoName := path.Base(repoURL)
	return repoName + "-" + version
}

// stringList converts a slice of strings to a BUILD list expression.
func stringList(strings []string) *build.ListExpr {
	var list []build.Expr
	for _, i := range strings {
		list = append(list, &build.StringExpr{Value: i})
	}
	return &build.ListExpr{List: list}
}

// rewriteGitRepos turns git_repository rules to http_archive rules.
func rewriteGitRepos(ws *build.File) {
	kinds := []string{
		"git_repository",
		"new_git_repository",
	}
	for _, kind := range kinds {
		for _, r := range ws.Rules(kind) {
			urls, err := archiveURLs(r)
			if err != nil {
				// not a github repo
				continue
			}
			r.SetKind(strings.Replace(r.Kind(), "git_repository", "http_archive", 1))
			r.SetAttr("strip_prefix", &build.StringExpr{Value: archivePrefix(r)})
			r.SetAttr("urls", stringList(urls))
			r.DelAttr("commit")
			r.DelAttr("remote")
			r.DelAttr("tag")
			if dryRun {
				fmt.Println("convert", r.Name(), "to http_archive")
			}
		}
	}
}

// getURLs gets URLs from the urls attribute and falls back to the deprecated
// url attribute.
func getURLs(r *build.Rule) []string {
	urls := r.AttrStrings("urls")
	if urls != nil {
		return urls
	}
	return []string{r.AttrString("url")}
}

// hashArchive downloads and calculates the SHA256 hash of an archive.
func hashArchive(r *build.Rule) error {
	if r.AttrString("sha256") != "" {
		// There's already a hash, so don't download it.
		return nil
	}
	// TODO(rodrigoq): add retries and mirrors if we start seeing errors
	resp, err := http.Get(getURLs(r)[0])
	defer resp.Body.Close()
	if err != nil {
		return err
	}
	h := sha256.New()
	_, err = io.Copy(h, resp.Body)
	if err != nil {
		return err
	}
	hexhash := hex.EncodeToString(h.Sum(nil))
	r.SetAttr("sha256", &build.StringExpr{Value: hexhash})
	if dryRun {
		log.Println("added sha256 for", r.Name())
	}
	return err
}

func hasMirror(r *build.Rule) bool {
	for _, u := range getURLs(r) {
		if isMirror(u) {
			return true
		}
	}
	return false
}

func getMirrorURL(rawurl string) (string, error) {
	url, err := url.Parse(rawurl)
	if err != nil {
		return "", err
	}
	url.Path = url.Hostname() + url.Path
	url.Host = "mirror.bazel.build"
	url.Scheme = "https"
	return url.String(), nil
}

// mirrorArchive checks if the archive is mirrored on mirror.bazel.build, and
// adds the mirror if so. It returns an error if the file is not mirrored.
func mirrorArchive(r *build.Rule) error {
	sourceURL, err := getSourceURL(r)
	if err != nil {
		return err
	}
	mirrorURL, err := getMirrorURL(sourceURL)
	if err != nil {
		return err
	}
	resp, err := http.Head(mirrorURL)
	if err != nil {
		return err
	}
	if resp.StatusCode == http.StatusForbidden {
		// mirror.bazel.build gives 403 for missing files.
		return errors.New("no archive on mirror")
	} else if resp.StatusCode != http.StatusOK {
		return errors.New(resp.Status)
	}
	urls := append([]string{mirrorURL}, getURLs(r)...)
	r.SetAttr("urls", stringList(urls))
	r.DelAttr("url")
	if dryRun {
		log.Printf("added mirror for %s:\n    %v", r.Name(), mirrorURL)
	}
	return nil
}

// rewriteArchive adds a hash to the archive if possible.
func rewriteArchive(r *build.Rule) error {
	if !hasMirror(r) {
		if err := mirrorArchive(r); err != nil {
			log.Printf("failed to mirror %s: %v", r.Name(), err)
		}
	}
	return hashArchive(r)
}

// startWorkers starts worker goroutines to rewrite HTTP archives.
func startWorkers(tasks <-chan *build.Rule, rewrittenArchives chan<- *build.Rule) {
	// TODO(rodrigoq): determine if this should be reduced when
	// runtime.NumCPU() is small - maybe it doesn't matter if the workers
	// are IO-bound.
	numWorkers := 8
	var wg sync.WaitGroup
	wg.Add(numWorkers)
	go func() {
		wg.Wait()
		close(rewrittenArchives)
	}()
	for i := 0; i < numWorkers; i++ {
		go func() {
			defer wg.Done()
			for r := range tasks {
				err := rewriteArchive(r)
				if err != nil {
					fmt.Println("error rewriting", r.Name(), err)
				} else {
					rewrittenArchives <- r
				}
			}
		}()
	}
}

// rewriteArchives calls rewriteArchive on http_archive rules with a worker
// pool.
func rewriteArchives(ws *build.File) {
	tasks := make(chan *build.Rule, len(ws.Stmt))
	rewrittenArchives := make(chan *build.Rule)
	startWorkers(tasks, rewrittenArchives)
	kinds := []string{
		"http_archive",
		"new_http_archive",
	}
	for _, kind := range kinds {
		for _, r := range ws.Rules(kind) {
			tasks <- r
		}
	}
	close(tasks)

	// TODO(rodrigoq): add progress output during download
	// This empty iteration blocks until all archives are rewritten and the
	// channel is closed.
	for range rewrittenArchives {
	}
}

// fixWorkspaceCmd represents the fixWorkspace command.
var fixWorkspaceCmd = &cobra.Command{
	Use:   "fixWorkspace",
	Short: "Applies automatic improvements to the WORKSPACE file",
	Long: `fixWorkspace transforms git_repository rules to http_archive
and adds sha256s where missing.`,
	RunE: func(cmd *cobra.Command, args []string) error {
		if err := checkWorkspacePath(); err != nil {
			return err
		}
		workspaceFile := filepath.Join(workspace, "WORKSPACE")
		ws, err := loadWorkspace(workspaceFile)
		if err != nil {
			return err
		}
		rewriteGitRepos(ws)
		rewriteArchives(ws)
		if dryRun {
			return nil
		}
		return saveWorkspace(workspaceFile, ws)
	},
}

func init() {
	RootCmd.AddCommand(fixWorkspaceCmd)
}
