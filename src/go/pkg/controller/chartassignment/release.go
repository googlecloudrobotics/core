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
	"encoding/hex"
	"io"
	"io/ioutil"
	"log"
	"os"
	"path/filepath"
	"strings"
	"sync"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"github.com/googlecloudrobotics/core/src/go/pkg/synk"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"
	core "k8s.io/api/core/v1"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"k8s.io/apimachinery/pkg/util/yaml"
	"k8s.io/client-go/rest"
	"k8s.io/client-go/tools/record"
	"k8s.io/helm/pkg/chartutil"
	"k8s.io/helm/pkg/downloader"
	"k8s.io/helm/pkg/getter"
	"k8s.io/helm/pkg/helm/helmpath"
	"k8s.io/helm/pkg/proto/hapi/chart"
	"k8s.io/helm/pkg/renderutil"
	"k8s.io/helm/pkg/repo"
)

// releases is a cache of releases currently handled.
type releases struct {
	recorder record.EventRecorder
	synk     synk.Interface

	mtx sync.Mutex
	m   map[string]*release
}

func newReleases(cfg *rest.Config, rec record.EventRecorder) (*releases, error) {
	synk, err := synk.NewForConfig(cfg)
	if err != nil {
		return nil, err
	}
	return &releases{
		recorder: rec,
		m:        map[string]*release{},
		synk:     synk,
	}, nil
}

// release is a cache object which acts as a proxy for Synk ResourceSets.
type release struct {
	name       string
	synk       synk.Interface
	recorder   record.EventRecorder
	actorc     chan func()
	generation int64 // last deployed generation.

	mtx    sync.Mutex
	status releaseStatus
}

type releaseStatus struct {
	phase apps.ChartAssignmentPhase
	err   error // last encountered error
	retry bool  // whether deployment should be retried.
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

// ensureUpdated ensures that the ChartAssignment is installed as a Synk
// ResourceSet.
// It returns true if it could initiate an update successfully.
func (rs *releases) ensureUpdated(as *apps.ChartAssignment) bool {
	r := rs.add(as.Name)
	status, _ := rs.status(as.Name)

	// If the last generation we deployed matches the provided one, there's
	// nothing to do. Unless the previous update set the retry flag due to
	// a transient error.
	// For a fresh release object, a first update will always happen as
	// r.generation is 0 and resource generations start at 1.
	if r.generation == as.Generation && !status.retry {
		return true
	}
	asCopy := as.DeepCopy()
	started := r.start(func() { r.update(asCopy) })
	if started {
		r.generation = as.Generation
	}
	return started
}

// ensureDeleted ensures that deletion of the release is run.
// It returns true if it could initiate deletion successfully.
func (rs *releases) ensureDeleted(as *apps.ChartAssignment) bool {
	r := rs.add(as.Name)
	asCopy := as.DeepCopy()
	return r.start(func() { r.delete(asCopy) })
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
	r.status.err = nil
	r.status.retry = false
	r.mtx.Unlock()
}

func (r *release) setFailed(err error, retry bool) {
	r.mtx.Lock()
	if !retry {
		// We only update the phase for non-retriable errors. This mitigates a
		// race condition between ensureUpdated, which sets phase=Updating when
		// retrying, and setStatus, which reads either the old phase or Updating
		// and copies it to the chartassignment status.
		r.status.phase = apps.ChartAssignmentPhaseFailed
	}
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

	if err := r.synk.Delete(context.Background(), as.Name); err != nil {
		r.recorder.Event(as, core.EventTypeWarning, "Failure", err.Error())
		r.setFailed(errors.Wrap(err, "delete release"), synk.IsTransientErr(err))
	}
	r.recorder.Event(as, core.EventTypeNormal, "Success", "chart deleted successfully")
	r.setPhase(apps.ChartAssignmentPhaseDeleted)
	// Reset last deployed generation to 0 as the ChartAssignment will be deleted
	// and its generation start at 1 again if it is re-created.
	r.generation = 0
}

func (r *release) update(as *apps.ChartAssignment) {
	r.setPhase(apps.ChartAssignmentPhaseLoadingChart)
	resources, retry, err := loadAndExpandChart(as)
	if err != nil {
		r.recorder.Event(as, core.EventTypeWarning, "Failure", err.Error())
		r.setFailed(err, retry)
		return
	}

	r.setPhase(apps.ChartAssignmentPhaseUpdating)
	r.recorder.Event(as, core.EventTypeNormal, "UpdateChart", "update chart")

	opts := &synk.ApplyOptions{
		Namespace:        as.Spec.NamespaceName,
		EnforceNamespace: true,
		Log: func(r *unstructured.Unstructured, action apps.ResourceAction, status, msg string) {
			if status == synk.StatusSuccess {
				return
			}
			log.Printf("[%s] %s %s/%s %s: %s\n",
				strings.ToUpper(status), action,
				r.GetAPIVersion(), r.GetKind(),
				r.GetName(), msg)
		},
	}
	spanContext := trace.SpanContext{}
	if tid, found := as.GetAnnotations()["cloudrobotics.com/trace-id"]; found {
		if _, err := hex.Decode(spanContext.TraceID[:], []byte(tid)); err != nil {
			log.Printf("Error: decoding TraceID: %v, %v", tid, err)
		}
	}
	ctx, span := trace.StartSpanWithRemoteParent(context.Background(), "Apply "+as.Name, spanContext)
	_, err = r.synk.Apply(ctx, as.Name, opts, resources...)
	span.End()
	if err != nil {
		r.recorder.Event(as, core.EventTypeWarning, "Failure", err.Error())
		r.setFailed(err, synk.IsTransientErr(err))
		return
	}
	r.recorder.Event(as, core.EventTypeNormal, "Success", "chart updated successfully")
	r.setPhase(apps.ChartAssignmentPhaseSettled)
}

func loadAndExpandChart(as *apps.ChartAssignment) ([]*unstructured.Unstructured, bool, error) {
	c, values, err := loadChart(&as.Spec.Chart)
	if err != nil {
		return nil, true, err
	}
	// Expand chart.
	manifests, err := renderutil.Render(c, &chart.Config{Raw: values}, renderutil.Options{
		ReleaseOptions: chartutil.ReleaseOptions{
			Name:      as.Name,
			Namespace: as.Spec.NamespaceName,
			IsInstall: true,
		},
	})
	if err != nil {
		return nil, false, errors.Wrap(err, "render chart")
	}
	// TODO: consider giving the synk package first-class support for raw manifests
	// so that their decoding errors are fully surfaced in the ResourceSet. Otherwise,
	// common YAML errors will only be surfaced one-by-one, which is tedious to handle.
	res, err := decodeManifests(manifests)
	if err != nil {
		return nil, false, err
	}
	return res, false, nil
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

	// TODO: handle empty c.Values, cspec.Values
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
