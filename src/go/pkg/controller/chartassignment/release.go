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

package chartassignment

import (
	"bytes"
	"context"
	"encoding/base64"
	"io"
	"io/ioutil"
	"os"
	"path/filepath"
	"strings"
	"sync"

	"github.com/gogo/protobuf/proto"
	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"github.com/googlecloudrobotics/core/src/go/pkg/synk"
	"github.com/pkg/errors"
	core "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"k8s.io/apimachinery/pkg/util/yaml"
	"k8s.io/client-go/rest"
	record "k8s.io/client-go/tools/record"
	"k8s.io/helm/pkg/chartutil"
	"k8s.io/helm/pkg/downloader"
	"k8s.io/helm/pkg/getter"
	hclient "k8s.io/helm/pkg/helm"
	"k8s.io/helm/pkg/helm/helmpath"
	"k8s.io/helm/pkg/proto/hapi/chart"
	hrelease "k8s.io/helm/pkg/proto/hapi/release"
	"k8s.io/helm/pkg/renderutil"
	"k8s.io/helm/pkg/repo"
)

// releases is a cache of releases currently handled.
type releases struct {
	helm     hclient.Interface
	recorder record.EventRecorder
	synk     *synk.Synk
	useSynk  bool

	mtx sync.Mutex
	m   map[string]*release
}

func newReleases(cfg *rest.Config, helm hclient.Interface, rec record.EventRecorder, useSynk bool) (*releases, error) {
	synk, err := synk.NewForConfig(cfg)
	if err != nil {
		return nil, err
	}
	return &releases{
		helm:     helm,
		recorder: rec,
		m:        map[string]*release{},
		synk:     synk,
		useSynk:  useSynk,
	}, nil
}

// release is a cache object which acts as a proxy for Helm releases.
type release struct {
	name       string
	helm       hclient.Interface
	synk       *synk.Synk
	recorder   record.EventRecorder
	actorc     chan func()
	generation int64 // last deployed generation.

	mtx    sync.Mutex
	status releaseStatus
}

type releaseStatus struct {
	phase       apps.ChartAssignmentPhase
	err         error  // last encountered error
	retry       bool   // whether deployment should be retried.
	revision    int32  // current Helm revision
	description string // current Helm description
}

// status returns the current phase and error of the release. ok is false
// if the release does not exist in the cache.
func (rs *releases) status(name string) (releaseStatus, bool) {
	rs.mtx.Lock()
	r, ok := rs.m[name]
	rs.mtx.Unlock()
	if !ok {
		return releaseStatus{}, false
	}
	r.mtx.Lock()
	defer r.mtx.Unlock()
	return r.status, true
}

// add a release to the cache with an initial phase.
func (rs *releases) add(name string) *release {
	rs.mtx.Lock()
	defer rs.mtx.Unlock()

	r, ok := rs.m[name]
	if ok {
		return r
	}
	r = &release{
		name:     name,
		helm:     rs.helm,
		synk:     rs.synk,
		recorder: rs.recorder,
		actorc:   make(chan func()),
	}
	r.status.phase = apps.ChartAssignmentPhaseAccepted
	rs.m[name] = r
	// Start applying updates in the background.
	go r.run()
	return r
}

// ensureUpdated ensures that the ChartAssignment is installed as a Helm release
// or Synk ResourceSet.
// It returns true if it could initiate an update successfully.
func (rs *releases) ensureUpdated(as *apps.ChartAssignment) bool {
	r := rs.add(as.Name)
	status, _ := rs.status(as.Name)

	// If the last generation we deployed matches the provided one, there's
	// nothing to do. Unless the previous update set the retry flag due to
	// a transient error.
	// For a fresh release object, a first update will always happen as r.generatio
	// is 0 and resource generations start at 1.
	if r.generation == as.Generation && !status.retry {
		return true
	}
	var started bool
	if rs.useSynk {
		started = r.start(func() {
			r.deleteHelm(as) // Best effort deletion when switching.
			r.updateSynk(as)
		})
	} else {
		started = r.start(func() {
			r.deleteSynk(as) // Best effort deletion when switching.
			r.updateHelm(as)
		})
	}
	if started {
		r.generation = as.Generation
	}
	return started
}

// ensureDeleted ensures that deletion of the release is run.
// It returns true if it could intiiate deletion successfully.
func (rs *releases) ensureDeleted(as *apps.ChartAssignment) bool {
	r := rs.add(as.Name)
	return r.start(func() { r.delete(as) })
}

// run all functions sent on the actor channel in sequence.
func (r *release) run() {
	for f := range r.actorc {
		f()
	}
}

// start tries to launch f on the worker goroutine.
// If there's already a function running, it immediately returns false.
func (r *release) start(f func()) bool {
	select {
	case r.actorc <- f:
		return true
	default:
	}
	return false
}

func (r *release) setPhase(p apps.ChartAssignmentPhase) {
	r.mtx.Lock()
	r.status.phase = p
	r.mtx.Unlock()
}

func (r *release) setFailed(err error, retry bool) {
	r.mtx.Lock()
	r.status.phase = apps.ChartAssignmentPhaseFailed
	r.status.err = err
	r.status.retry = retry
	r.mtx.Unlock()
}

func (r *release) delete(as *apps.ChartAssignment) {
	r.mtx.Lock()
	currentPhase := r.status.phase
	r.mtx.Unlock()
	if currentPhase == apps.ChartAssignmentPhaseDeleted {
		return
	}

	r.setPhase(apps.ChartAssignmentPhaseDeleting)
	r.recorder.Event(as, core.EventTypeNormal, "DeleteChart", "deleting chart")

	err1 := r.deleteSynk(as)
	err2 := r.deleteHelm(as)
	if err1 == nil {
		err1 = err2
	}
	if err1 != nil {
		r.recorder.Event(as, core.EventTypeWarning, "Failure", err1.Error())
		r.setFailed(errors.Wrap(err1, "delete release"), synk.IsTransientErr(err1))
	}
	r.recorder.Event(as, core.EventTypeNormal, "Success", "chart deleted successfully")
	r.mtx.Lock()
	r.status.err = nil
	r.status.phase = apps.ChartAssignmentPhaseDeleted
	r.mtx.Unlock()
	// Reset last deployed generation to 0 as the ChartAssignment will be deleted
	// and its generation start at 1 again if it is re-created.
	r.generation = 0
}

func (r *release) deleteSynk(as *apps.ChartAssignment) error {
	return r.synk.Delete(context.Background(), as.Name)
}

func (r *release) deleteHelm(as *apps.ChartAssignment) error {
	_, err := r.helm.DeleteRelease(as.Name,
		hclient.DeletePurge(true),
		hclient.DeleteTimeout(tillerTimeout),
	)
	if err != nil && !strings.Contains(err.Error(), "not found") {
		return err
	}
	return nil
}

func (r *release) updateSynk(as *apps.ChartAssignment) {
	r.setPhase(apps.ChartAssignmentPhaseLoadingChart)

	c, vals, err := loadChart(&as.Spec.Chart)
	if err != nil {
		r.setFailed(err, true)
		return
	}
	// Expand chart.
	manifests, err := renderutil.Render(c, &chart.Config{Raw: vals}, renderutil.Options{
		ReleaseOptions: chartutil.ReleaseOptions{
			Name:      as.Name,
			Namespace: as.Spec.NamespaceName,
			IsInstall: true,
		},
	})
	if err != nil {
		r.setFailed(errors.Wrap(err, "render chart"), false)
		return
	}
	// TODO: consider giving the synk package first-class support for raw manifests
	// so that their decoding errors are fully surfaced in the ResourceSet. Otherwise,
	// common YAML errors will only be surfaced one-by-one, which is tedious to handle.
	resources, err := decodeManifests(manifests)
	if err != nil {
		r.setFailed(err, false)
		return
	}
	r.setPhase(apps.ChartAssignmentPhaseUpdating)
	r.recorder.Event(as, core.EventTypeNormal, "UpdatChart", "update chart")

	opts := &synk.ApplyOptions{
		Namespace:        as.Spec.NamespaceName,
		EnforceNamespace: true,
	}
	_, err = r.synk.Apply(context.Background(), as.Name, opts, resources...)
	if err != nil {
		r.recorder.Event(as, core.EventTypeWarning, "Failure", err.Error())
		r.setFailed(err, synk.IsTransientErr(err))
		return
	}
	r.recorder.Event(as, core.EventTypeNormal, "Success", "chart upgraded successfully")

	r.mtx.Lock()
	r.status.phase = apps.ChartAssignmentPhaseSettled
	r.status.err = nil
	r.status.retry = false
	r.mtx.Unlock()
}

func (r *release) updateHelm(as *apps.ChartAssignment) {
	r.setPhase(apps.ChartAssignmentPhaseLoadingChart)

	chart, vals, err := loadChart(&as.Spec.Chart)
	if err != nil {
		r.setFailed(err, true)
		return
	}
	deployErr := r.deploy(as, chart, vals)
	// A deploy error may have been triggered before or after the new release
	// revision was applied. The only way to find out is checking the most
	// recent release contents.
	// If the contents are up-to-date we consider the failure permanent, otherwise
	// transient (e.g. due to Tiller connectivity issues).
	// Some states we consider permanent may be recoverable through retries but
	// only after some user intervention, e.g. when resources are conflicting.
	// Similarly, some failures before updating release contents may be permanent
	// but we hope to generally catch them early when loading the chart above.
	rel, err := r.helm.ReleaseContent(as.Name)
	if err != nil {
		// Deployment didn't go through, return the deploy error.
		if strings.Contains(err.Error(), "not found") {
			r.setFailed(deployErr, true)
			return
		}
		r.setFailed(err, true)
		return
	}
	r.mtx.Lock()
	r.status.revision = rel.Release.Version
	r.status.description = rel.Release.Info.Description
	r.mtx.Unlock()

	if deployErr != nil {
		r.setFailed(deployErr, !releaseEquals(rel.Release, chart, vals))
		return
	}
	r.mtx.Lock()
	r.status.phase = apps.ChartAssignmentPhaseSettled
	r.status.err = nil
	r.status.retry = false
	r.mtx.Unlock()
}

func releaseEquals(r *hrelease.Release, c *chart.Chart, vals string) bool {
	return proto.Equal(r.Chart, c) && r.Config.Raw == vals
}

func (r *release) deploy(as *apps.ChartAssignment, chart *chart.Chart, vals string) error {
	_, err := r.helm.ReleaseContent(as.Name)
	if err != nil {
		if !strings.Contains(err.Error(), "not found") {
			return errors.Wrap(err, "get release content")
		}
		// First instance of this release for the namespace.
		r.setPhase(apps.ChartAssignmentPhaseInstalling)

		r.recorder.Event(as, core.EventTypeNormal, "InstallChart", "installing chart for the first time")
		_, err := r.helm.InstallReleaseFromChart(chart, as.Spec.NamespaceName,
			hclient.ReleaseName(as.Name),
			hclient.InstallTimeout(tillerTimeout),
			hclient.ValueOverrides([]byte(vals)),
		)
		if err != nil {
			r.recorder.Event(as, core.EventTypeWarning, "Failure", err.Error())
			return errors.Wrap(err, "chart installation failed")
		}
		r.recorder.Event(as, core.EventTypeNormal, "Success", "chart installed successfully")
		return nil
	}
	// Update to an existing release.
	r.setPhase(apps.ChartAssignmentPhaseUpdating)
	r.recorder.Event(as, core.EventTypeNormal, "UpgradeChart", "upgrade chart")
	// Often upgrades only succeed when force is set to replace resources
	// that cannot be updated, e.g. services.
	// Upgrading with force set may result in obfuscated error messages.
	// Try upgrading regularly first and retry with force on failure.
	// Return the first error in any case.
	_, err = r.helm.UpdateReleaseFromChart(as.Name, chart,
		hclient.UpgradeTimeout(tillerTimeout),
		hclient.UpdateValueOverrides([]byte(vals)),
	)
	if err != nil {
		_, err2 := r.helm.UpdateReleaseFromChart(as.Name, chart,
			hclient.UpgradeForce(true),
			hclient.UpgradeTimeout(tillerTimeout),
			hclient.UpdateValueOverrides([]byte(vals)),
		)
		// Original error was fixed by using force, clear it.
		if err2 == nil {
			err = nil
		}
	}
	if err != nil {
		r.recorder.Event(as, core.EventTypeWarning, "Failure", err.Error())
		return errors.Wrap(err, "chart upgrade failed")
	}
	r.recorder.Event(as, core.EventTypeNormal, "Success", "chart upgraded successfully")
	return nil
}

func loadChart(cspec *apps.AssignedChart) (*chart.Chart, string, error) {
	var archive io.Reader
	var err error

	if cspec.Inline != "" {
		archive = base64.NewDecoder(base64.StdEncoding, strings.NewReader(cspec.Inline))
	} else {
		archive, err = fetchChartTar(cspec.Repository, cspec.Name, cspec.Version)
		if err != nil {
			return nil, "", errors.Wrap(err, "retrieve chart")
		}
	}
	c, err := chartutil.LoadArchive(archive)
	if err != nil {
		return nil, "", errors.Wrap(err, "load chart archive")
	}

	// Ensure charts in requirements.yaml are actually in packaged in.
	if req, err := chartutil.LoadRequirements(c); err == nil {
		if err := renderutil.CheckDependencies(c, req); err != nil {
			return nil, "", errors.Wrap(err, "check chart dependencies")
		}
	} else if err != chartutil.ErrRequirementsNotFound {
		return nil, "", errors.Wrap(err, "load chart requirements")
	}

	// Build the full set of values including the default ones. Even though
	// they are part of the chart, they are ignored if we don't provide
	// them explicitly.
	vals, err := chartutil.ReadValues([]byte(c.Values.Raw))
	if err != nil {
		return nil, "", errors.Wrap(err, "reading chart values")
	}
	vals.MergeInto(chartutil.Values(cspec.Values)) // ChartAssignment values.

	valsRaw, err := vals.YAML()
	if err != nil {
		return nil, "", errors.Wrap(err, "encode values")
	}
	return c, valsRaw, nil
}

func fetchChartTar(repoURL, name, version string) (io.Reader, error) {
	c := downloader.ChartDownloader{
		Getters: getter.Providers{
			{Schemes: []string{"http", "https"}, New: newHTTPGetter},
		},
		HelmHome: helmpath.Home(os.ExpandEnv("$HOME/.helm")),
		Out:      os.Stderr,
		Keyring:  os.ExpandEnv("$HOME/.gnupg/pubring.gpg"),
		Verify:   downloader.VerifyIfPossible,
	}

	dir, err := ioutil.TempDir("", name)
	if err != nil {
		return nil, err
	}
	defer os.RemoveAll(dir)

	// This is only called when we fetch by repo URL rather than simply
	// by e.g. stable/postgresql.
	chartURL, err := repo.FindChartInRepoURL(repoURL, name, version, "", "", "", c.Getters)
	if err != nil {
		return nil, err
	}
	// NOTE(fabxc): Since we provide a full chartURL, DownloadTo will pull from exactly
	// that URL. It will however check for repos in $HOME/.helm to determine
	// whether it should do this with special certificates for that domain.
	// (The same cert files we left blank above.)
	// We might just want to implement this ourselves once we know what auth
	// strategies we want to support and how users can configure them.
	filename, _, err := c.DownloadTo(chartURL, version, dir)
	if err != nil {
		return nil, err
	}
	b, err := ioutil.ReadFile(filename)
	if err != nil {
		return nil, err
	}
	return bytes.NewReader(b), nil
}

// newHTTPGetter return a Helm chart getter for HTTP(s) repositories.
func newHTTPGetter(url, certFile, keyFile, caFile string) (getter.Getter, error) {
	return getter.NewHTTPGetter(url, certFile, keyFile, caFile)
}

func decodeManifests(manifests map[string]string) (res []*unstructured.Unstructured, err error) {
	for k, v := range manifests {
		// Sometimes README.md or NOTES.txt files make it into the template directory.
		// Filter files by extension.
		switch filepath.Ext(k) {
		case ".json", ".yml", ".yaml":
		default:
			continue
		}
		dec := yaml.NewYAMLOrJSONDecoder(strings.NewReader(v), 4096)
		for i := 0; ; i++ {
			var u unstructured.Unstructured
			if err := dec.Decode(&u); err == io.EOF {
				break
			} else if err != nil {
				return nil, errors.Wrapf(err, "decode manifest %d in %q", i, k)
			}
			res = append(res, &u)
		}
	}
	return res, nil
}
