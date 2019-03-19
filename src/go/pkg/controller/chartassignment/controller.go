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
	"fmt"
	"io"
	"io/ioutil"
	"log"
	"net/http"
	"os"
	"regexp"
	"strconv"
	"strings"
	"time"

	"github.com/gogo/protobuf/proto"
	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"github.com/googlecloudrobotics/core/src/go/pkg/gcr"
	admissionregistration "k8s.io/api/admissionregistration/v1beta1"
	core "k8s.io/api/core/v1"
	k8serrors "k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/api/validation"
	meta "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/runtime/serializer"
	record "k8s.io/client-go/tools/record"
	"k8s.io/helm/pkg/chartutil"
	"k8s.io/helm/pkg/downloader"
	"k8s.io/helm/pkg/getter"
	hclient "k8s.io/helm/pkg/helm"
	"k8s.io/helm/pkg/helm/helmpath"
	"k8s.io/helm/pkg/proto/hapi/release"
	"k8s.io/helm/pkg/renderutil"
	"k8s.io/helm/pkg/repo"
	kclient "sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/controller"
	"sigs.k8s.io/controller-runtime/pkg/handler"
	"sigs.k8s.io/controller-runtime/pkg/manager"
	"sigs.k8s.io/controller-runtime/pkg/reconcile"
	"sigs.k8s.io/controller-runtime/pkg/source"
	"sigs.k8s.io/controller-runtime/pkg/webhook"
	"sigs.k8s.io/controller-runtime/pkg/webhook/admission"
	"sigs.k8s.io/controller-runtime/pkg/webhook/admission/builder"
	admissiontypes "sigs.k8s.io/controller-runtime/pkg/webhook/admission/types"
)

// In-cluster hostname of tiller in a standard installation.
const DefaultTillerHost = "tiller-deploy.kube-system.svc:44134"

// Add adds a controller and validation webhook for the ChartAssignment resource type
// to the manager and server.
// Handled ChartAssignments are filtered by the provided cluster.
func Add(mgr manager.Manager, cluster, tillerHost string) error {
	if tillerHost == "" {
		tillerHost = DefaultTillerHost
	}
	c, err := controller.New("chartassignment", mgr, controller.Options{
		Reconciler: &Reconciler{
			kube:     mgr.GetClient(),
			helm:     hclient.NewClient(hclient.Host(tillerHost)),
			recorder: mgr.GetRecorder("chartassignment-controller"),
			cluster:  cluster,
		},
	})
	if err != nil {
		return err
	}
	err = c.Watch(
		&source.Kind{Type: &apps.ChartAssignment{}},
		&handler.EnqueueRequestForObject{},
	)
	if err != nil {
		return err
	}
	return nil
}

// Reconciler provides an idempotent function that brings the cluster into a
// state consistent with the specification of a ChartAssignment.
type Reconciler struct {
	kube     kclient.Client
	helm     hclient.Interface
	recorder record.EventRecorder
	cluster  string // Cluster for which to handle AppAssignments.
}

// Reconcile creates and updates a Helm release for the given chart assignment.
// It rolls back releases to the previous revision if an upgrade failed.
// It continously requeues the ChartAssignment for reconciliation to monitor the
// status of the Helm release.
func (r *Reconciler) Reconcile(req reconcile.Request) (reconcile.Result, error) {
	ctx := context.TODO()

	var as apps.ChartAssignment
	err := r.kube.Get(ctx, req.NamespacedName, &as)

	if as.Spec.ClusterName != r.cluster {
		return reconcile.Result{}, nil
	}
	if k8serrors.IsNotFound(err) {
		// Assignment was already deleted. We did all required cleanup
		// when removing the finalizer. Thus, there's nothing to do.
		return reconcile.Result{}, nil
	} else if err != nil {
		return reconcile.Result{}, fmt.Errorf("getting ChartAssignment %q failed: %s", req, err)
	}
	return r.reconcile(ctx, &as)
}

const (
	// The finalizer that's applied to assignments to block their garbage collection
	// until the Helm release is deleted.
	finalizer = "helm.apps.cloudrobotics.com"
	// Requeue interval when the underlying Helm release is not in a stable state yet.
	requeueFast = 3 * time.Second
	// Requeue interval after the underlying Helm release reached a stable state.
	requeueSlow = time.Minute
	// Timeout seconds after which Tiller should abort any attempted release operations.
	tillerTimeout = 600
)

func (r *Reconciler) ensureNamespace(ctx context.Context, as *apps.ChartAssignment) (*core.Namespace, error) {
	// Create application namespace if it doesn't exist.
	var ns core.Namespace
	err := r.kube.Get(ctx, kclient.ObjectKey{Name: as.Spec.NamespaceName}, &ns)

	if err != nil && !k8serrors.IsNotFound(err) {
		return nil, fmt.Errorf("getting Namespace %q failed: %s", as.Spec.NamespaceName, err)
	}
	if ns.DeletionTimestamp != nil {
		return nil, fmt.Errorf("namespace %q is marked for deletion, skipping", as.Spec.NamespaceName)
	}

	createNamespace := k8serrors.IsNotFound(err)
	ns.Name = as.Spec.NamespaceName

	// Add ourselves to the owners if we aren't already.
	_true := true
	added := setOwnerReference(&ns.ObjectMeta, meta.OwnerReference{
		APIVersion:         as.APIVersion,
		Kind:               as.Kind,
		Name:               as.Name,
		UID:                as.UID,
		BlockOwnerDeletion: &_true,
	})
	if !added {
		return &ns, nil
	}
	if createNamespace {
		return &ns, r.kube.Create(ctx, &ns)
	}
	return &ns, r.kube.Update(ctx, &ns)
}

// ensureServiceAccount makes sure we have an image pull secret for gcr.io inside the apps namespace
// and the default service account configured to use it. This is needed to make apps work that
// reference images from a private container registry.
// TODO(ensonic): Put this behind a flag to only do this as needed.
func (r *Reconciler) ensureServiceAccount(ctx context.Context, ns *core.Namespace, as *apps.ChartAssignment) error {
	if r.cluster == "cloud" {
		// We don't need any of this for cloud charts.
		return nil
	}

	// Copy imagePullSecret from 'default' namespace, since service accounts cannot reference
	// secrets in other namespaces.
	var secret core.Secret
	err := r.kube.Get(ctx, kclient.ObjectKey{Namespace: as.Spec.NamespaceName, Name: gcr.SecretName}, &secret)
	if k8serrors.IsNotFound(err) {
		err = r.kube.Get(ctx, kclient.ObjectKey{Namespace: "default", Name: gcr.SecretName}, &secret)
		if k8serrors.IsNotFound(err) {
			log.Printf("Failed to get Secret \"default:%s\" (this is expected when simulating a robot on GKE)", gcr.SecretName)
			return nil
		} else if err != nil {
			return fmt.Errorf("getting Secret \"default:%s\" failed: %s", gcr.SecretName, err)
		}
		// Don't reuse full metadata in created secret.
		secret.ObjectMeta = meta.ObjectMeta{
			Namespace: ns.Name,
			Name:      gcr.SecretName,
		}
		err = r.kube.Create(ctx, &secret)
		if err != nil {
			return fmt.Errorf("creating Secret \"%s:%s\" failed: %s", as.Spec.NamespaceName, gcr.SecretName, err)
		}
	}

	// Configure the default service account in the namespace.
	var sa core.ServiceAccount
	err = r.kube.Get(ctx, kclient.ObjectKey{Namespace: as.Spec.NamespaceName, Name: "default"}, &sa)
	if err != nil {
		return fmt.Errorf("getting ServiceAccount \"%s:default\" failed: %s", as.Spec.NamespaceName, err)
	}

	// Only add the secret once.
	ips := core.LocalObjectReference{Name: gcr.SecretName}
	found := false
	for _, s := range sa.ImagePullSecrets {
		if s == ips {
			found = true
			break
		}
	}
	if !found {
		sa.ImagePullSecrets = append(sa.ImagePullSecrets, ips)
	}
	return r.kube.Update(ctx, &sa)
}

// releaseFailed returns whether the most recent release failed and what the last
// successful revision was.
func releaseFailed(releases []*release.Release) (bool, int32) {
	if len(releases) == 0 {
		return false, 0
	}
	active := releases[0]

	if active.Info.Status.Code != release.Status_FAILED {
		return false, 0
	}
	// Search for first past release that did not fail and was not a rollback.
	for _, r := range releases[1:] {
		d := decodeReleaseDesc(r.Info.Description)
		if !d.failed && (d.action == releaseActionInstall || d.action == releaseActionUpgrade) {
			return true, r.Version
		}
	}
	return false, 0
}

func (r *Reconciler) reconcile(ctx context.Context, as *apps.ChartAssignment) (reconcile.Result, error) {
	// If we are scheduled for deletion, delete the Helm release and drop our
	// finalizer so garbage collection can continue.
	if as.DeletionTimestamp != nil {
		log.Printf("Ensure ChartAssignment %q cleanup", as.Name)

		if err := r.ensureDeleted(ctx, as); err != nil {
			return reconcile.Result{}, fmt.Errorf("ensure deleted: %s", err)
		}
		if err := r.setStatus(ctx, as); err != nil {
			return reconcile.Result{}, fmt.Errorf("set status: %s", err)
		}
		// Requeue to track deletion progress.
		return reconcile.Result{Requeue: true, RequeueAfter: requeueSlow}, nil
	}

	ns, err := r.ensureNamespace(ctx, as)
	if err != nil {
		return reconcile.Result{}, fmt.Errorf("ensure namespace: %s", err)
	}
	if err := r.ensureServiceAccount(ctx, ns, as); err != nil {
		return reconcile.Result{}, fmt.Errorf("ensure service-account: %s", err)
	}
	// Ensure a finalizer on the ChartAssignment so we don't get deleted before
	// we've properly deleted the associated Helm release.
	if !stringsContain(as.Finalizers, finalizer) {
		as.Finalizers = append(as.Finalizers, finalizer)
		if err := r.kube.Update(ctx, as); err != nil {
			return reconcile.Result{}, err
		}
	}

	// Poll for status updates or try to rollback if the spec didn't change
	// and we are in a state where we don't want to reapply the chart.
	if as.Generation == as.Status.ObservedGeneration &&
		(inCondition(as, apps.ChartAssignmentConditionUpdated) ||
			inCondition(as, apps.ChartAssignmentConditionRolledBack) ||
			inCondition(as, apps.ChartAssignmentConditionMalformedChart)) {
		// Check for failure and try to rollback to the last good revision if the spec
		// has not been updated yet.
		history, err := r.helm.ReleaseHistory(as.Name, hclient.WithMaxHistory(100))
		if err != nil {
			return reconcile.Result{}, fmt.Errorf("fetch history: %s", err)
		}
		if failed, rev := releaseFailed(history.Releases); failed && rev >= 0 {
			if err := r.rollbackChart(as, rev); err != nil {
				return reconcile.Result{}, fmt.Errorf("rollback to revision %d failed: %s", rev, err)
			}
		} else {
			log.Printf("Update ChartAssignment %q status", as.Name)
		}
		// Either we are rolling back, still deploying the last revision, or in permanent
		// failure. Update status and enqueue to poll for status updates.
		if err := r.setStatus(ctx, as); err != nil {
			return reconcile.Result{}, fmt.Errorf("set status: %s", err)
		}
		return reconcile.Result{Requeue: true, RequeueAfter: requeueSlow}, nil
	}

	log.Printf("Apply chart for ChartAssignment %q", as.Name)

	updated, err := r.applyChart(as)

	updatedCond := core.ConditionFalse
	if updated {
		updatedCond = core.ConditionTrue
	}
	if _, ok := err.(errMalformedChart); ok {
		// We didn't update successfully but the malformed chart is a permanent
		// failure, so we won't reapply the chart next time.
		setCondition(as, apps.ChartAssignmentConditionMalformedChart, core.ConditionTrue, err.Error())
		setCondition(as, apps.ChartAssignmentConditionUpdated, updatedCond, "Malformed Chart")
	} else if err == errChartSkipped {
		setCondition(as, apps.ChartAssignmentConditionMalformedChart, core.ConditionFalse, "Chart OK")
		setCondition(as, apps.ChartAssignmentConditionUpdated, updatedCond, "")
	} else if err != nil {
		setCondition(as, apps.ChartAssignmentConditionMalformedChart, core.ConditionFalse, "Chart OK")
		setCondition(as, apps.ChartAssignmentConditionUpdated, updatedCond, err.Error())
	} else {
		setCondition(as, apps.ChartAssignmentConditionMalformedChart, core.ConditionFalse, "Chart OK")
		setCondition(as, apps.ChartAssignmentConditionUpdated, updatedCond, "Chart up to date")
	}

	if serr := r.setStatus(ctx, as); serr != nil {
		return reconcile.Result{}, fmt.Errorf("set status: %s", serr)
	}
	// If there was a chart apply error, don't enqueue with specific time to
	// fallback to exponential backoff.
	if err != nil {
		return reconcile.Result{Requeue: true}, nil
	}
	// Requeue quickly to update status as chart deployment progresses.
	return reconcile.Result{Requeue: true, RequeueAfter: requeueFast}, nil
}

// setStatus updates the status section of the ChartAssignment based on Helm's reported
// state. It returns true if a rollback should be attempted.
func (r *Reconciler) setStatus(ctx context.Context, as *apps.ChartAssignment) error {
	as.Status.ObservedGeneration = as.Generation

	history, err := r.helm.ReleaseHistory(as.Name, hclient.WithMaxHistory(100))
	if err != nil {
		if !strings.Contains(err.Error(), "not found") {
			// Creating the release likely failed previously and conditions
			// were set appropriately.
			return r.kube.Status().Update(ctx, as)
		}
		return fmt.Errorf("fetch history: %s", err)
	}
	if len(history.Releases) == 0 {
		return r.kube.Status().Update(ctx, as)
	}
	active := history.Releases[0]
	activeDesc := decodeReleaseDesc(active.Info.Description)

	as.Status.Phase = chartPhase(active.Info.Status.Code)

	// Most recent revision is what's currently deployed.
	as.Status.DeployedRevision = active.Version
	// If the current revision is a rollback, set the rollback revision. If we couldn't
	// determine it from history, it's set to 0 and will be left blank.
	if activeDesc.action == releaseActionRollback {
		as.Status.RollbackRevision = activeDesc.rollbackTo
		setCondition(as, apps.ChartAssignmentConditionRolledBack, core.ConditionTrue, "")
	} else {
		as.Status.RollbackRevision = 0
		setCondition(as, apps.ChartAssignmentConditionRolledBack, core.ConditionFalse, "")
	}
	// Last installed or upgraded release is what the user intended to run.
	foundDesiredRevision := false
	for _, r := range history.Releases {
		if a := decodeReleaseAction(r.Info.Description); a == releaseActionInstall || a == releaseActionUpgrade {
			as.Status.DesiredRevision = r.Version
			foundDesiredRevision = true
			break
		}
	}
	// If nothing in the history we fetched matches, we no longer know
	// what the desired revision was.
	if !foundDesiredRevision {
		as.Status.DesiredRevision = 0
		log.Printf("Release history for %q too short, could not determine desired revision", as.Name)
	}
	return r.kube.Status().Update(ctx, as)
}

// rollbackChart will rollback the chart to the previous revision if it exists
// and wasn't a rollback itself.
func (r *Reconciler) rollbackChart(as *apps.ChartAssignment, rev int32) error {
	r.recorder.Eventf(as, core.EventTypeNormal, "RollbackChart", "rolling back chart to revision %d", rev)

	_, err := r.helm.RollbackRelease(as.Name,
		hclient.RollbackForce(true),
		hclient.RollbackTimeout(tillerTimeout),
		hclient.RollbackVersion(rev),
	)
	if err != nil {
		r.recorder.Eventf(as, core.EventTypeWarning, "Failure", "chart rollback failed: %s", err)
	} else {
		r.recorder.Event(as, core.EventTypeNormal, "Success", "chart rolled back successfully")
	}
	return err
}

type errMalformedChart struct {
	error
}

var errChartSkipped = fmt.Errorf("skipped chart")

// applyChart installs or updates the chart. It returns true if the Chart could be applied
// to helm irrespective of whether its deployment failed.
// It returns an errMalformedChart, if the chart was broken so that deployment could
// not even be attempted.
func (r *Reconciler) applyChart(as *apps.ChartAssignment) (updated bool, err error) {
	var (
		cspec   = as.Spec.Chart
		archive io.Reader
	)
	if cspec.Inline != "" {
		archive = base64.NewDecoder(base64.StdEncoding, strings.NewReader(cspec.Inline))
	} else {
		archive, err = r.fetchChartTar(cspec.Repository, cspec.Name, cspec.Version)
		if err != nil {
			return false, errMalformedChart{fmt.Errorf("retrieving chart failed: %s", err)}
		}
	}
	c, err := chartutil.LoadArchive(archive)
	if err != nil {
		return false, errMalformedChart{fmt.Errorf("loading chart failed: %s", err)}
	}

	// Ensure charts in requirements.yaml are actually in packaged in.
	if req, err := chartutil.LoadRequirements(c); err == nil {
		if err := renderutil.CheckDependencies(c, req); err != nil {
			return false, errMalformedChart{fmt.Errorf("chart dependency error: %s", err)}
		}
	} else if err != chartutil.ErrRequirementsNotFound {
		return false, errMalformedChart{fmt.Errorf("cannot load requirements: %v", err)}
	}

	// Build the full set of values including the default ones. Even though
	// they are part of the chart, they are ignored if we don't provide
	// them explicitly.
	vals, err := chartutil.ReadValues([]byte(c.Values.Raw))
	if err != nil {
		return false, errMalformedChart{fmt.Errorf("reading chart values failed: %s", err)}
	}
	vals.MergeInto(chartutil.Values(as.Spec.Chart.Values)) // ChartAssignment values.

	valsRaw, err := vals.YAML()
	if err != nil {
		return false, errMalformedChart{fmt.Errorf("encoding values failed: %s", err)}
	}

	// Install if the release doesn't exist yet, update otherwise.
	var applyErr error

	cur, err := r.helm.ReleaseContent(as.Name)
	if err != nil {
		if !strings.Contains(err.Error(), "not found") {
			return false, fmt.Errorf("getting release status failed: %s", err)
		}
		r.recorder.Event(as, core.EventTypeNormal, "InstallChart", "installing chart for the first time")
		// First instance of this release for the namespace.
		_, applyErr = r.helm.InstallReleaseFromChart(c, as.Spec.NamespaceName,
			hclient.ReleaseName(as.Name),
			hclient.InstallTimeout(tillerTimeout),
			hclient.ValueOverrides([]byte(valsRaw)),
		)
		if applyErr != nil {
			applyErr = fmt.Errorf("chart installation failed: %s", applyErr)
			r.recorder.Event(as, core.EventTypeWarning, "Failure", applyErr.Error())
		} else {
			r.recorder.Event(as, core.EventTypeNormal, "Success", "chart installed successfully")
		}
	} else if proto.Equal(cur.Release.Chart, c) && cur.Release.Config.Raw == valsRaw {
		// Skip upgrade if contents didn't change. This allows us to retry applyChart
		// on any kind of error without creating an indefinite amount of new releases.
		r.recorder.Event(as, core.EventTypeNormal, "Skipping", "chart did not change")
		return true, errChartSkipped
	} else {
		r.recorder.Event(as, core.EventTypeNormal, "UpgradeChart", "upgrade chart")

		_, applyErr = r.helm.UpdateReleaseFromChart(as.Name, c,
			hclient.UpgradeForce(true),
			hclient.UpgradeTimeout(tillerTimeout),
			hclient.UpdateValueOverrides([]byte(valsRaw)),
		)
		if applyErr != nil {
			applyErr = fmt.Errorf("chart upgrade failed: %s", applyErr)
			r.recorder.Event(as, core.EventTypeWarning, "Failure", applyErr.Error())
		} else {
			r.recorder.Event(as, core.EventTypeNormal, "Success", "chart upgraded successfully")
		}
	}
	// Check Helm again whether the update went through.
	rel, err := r.helm.ReleaseContent(as.Name)
	if err != nil {
		if strings.Contains(err.Error(), "not found") {
			return false, applyErr
		}
		return false, fmt.Errorf("get release content: %s", err)
	}
	if !proto.Equal(rel.Release.Chart, c) || rel.Release.Config.Raw != valsRaw {
		return false, applyErr
	}
	return true, applyErr
}

// ensureDeleted ensures that the Helm release is deleted and the finalizer gets removed.
func (r *Reconciler) ensureDeleted(ctx context.Context, as *apps.ChartAssignment) error {
	_, err := r.helm.DeleteRelease(as.Name,
		hclient.DeletePurge(true),
		hclient.DeleteTimeout(tillerTimeout),
	)
	if err != nil && !strings.Contains(err.Error(), "not found") {
		return fmt.Errorf("deleting chart failed: %s", err)
	}
	if !stringsContain(as.Finalizers, finalizer) {
		return nil
	}
	as.Finalizers = stringsDelete(as.Finalizers, finalizer)
	if err := r.kube.Update(ctx, as); err != nil {
		return fmt.Errorf("update failed: %s", err)
	}
	return nil
}

func (r *Reconciler) fetchChartTar(repoURL, name, version string) (io.Reader, error) {
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

type releaseDesc struct {
	failed     bool
	action     releaseAction
	rollbackTo int32
}

type releaseAction string

const (
	releaseActionInstall  releaseAction = "install"
	releaseActionUpgrade                = "upgrade"
	releaseActionRollback               = "rollback"
	releaseActionDeleted                = "delete"
	releaseActionUnknown                = "unknown"
)

// decodeReleaseDesc decodes a Helm release description into the last performed action,
// whether it failed, and which revision it possibly rolled back to.
// See decodeReleaseAction for message format for different actions.
func decodeReleaseDesc(s string) releaseDesc {
	d := releaseDesc{
		action: decodeReleaseAction(s),
		failed: strings.Contains(s, "failed"),
	}
	// Decode revision that was rolled back to, e.g. "Rollback to 3".
	if d.action == releaseActionRollback {
		if parts := strings.Split(s, " "); len(parts) == 0 {
			log.Printf("Could not decode rollback revision from description %q", s)
		} else {
			rev, err := strconv.Atoi(parts[len(parts)-1])
			if err != nil {
				log.Printf("Could not decode rollback revision from description %q: %s", s, err)
			}
			d.rollbackTo = int32(rev)
		}
	}
	return d
}

// decodeReleaseAction decodes the action for a release from the string description
// message set by Helm.
// Setting an explicit structured JSON description when doing installs/rollbacks/upgrades
// did not work out since Helm rewrites the message on failure.
func decodeReleaseAction(s string) releaseAction {
	s = strings.ToLower(s)
	// A release was an installation when the message is:
	// 1. "Install complete."
	// 2. Release "<name>" failed: <error snippet>
	if strings.Contains(s, "install") || regexp.MustCompile(`release ".*" failed`).MatchString(s) {
		return releaseActionInstall
	}
	// A release was an upgrade when the message is:
	// 1. Upgrade complete.
	// 2. Upgrade "<name>" failed: <error snippet>
	if strings.Contains(s, "upgrade") {
		return releaseActionUpgrade
	}
	// A release was an upgrade when the message is:
	// 1. Rollback to 3.
	// 2. rollback "<name>" failed: <error_snippet>
	if strings.Contains(s, "rollback") {
		return releaseActionRollback
	}
	// a revision is described as deleted if it previously failed and was
	// replaced by a new version.
	if strings.Contains(s, "delete") || strings.Contains(s, "deletion") {
		return releaseActionDeleted
	}
	// There appear to be no descriptions before complete/fail, which hints
	// that the respective in-progress status codes like "installation pending"
	// are unused.
	log.Printf("Encountered unknown release action in description %q", s)
	return releaseActionUnknown
}

// decodeReleaseFailed returns true if the release description indicates that it failed.
// See decodeReleaseAction for possible descriptions.
func decodeReleaseFailed(s string) bool {
	return strings.Contains(s, "failed")
}

// newHTTPGetter return a Helm chart getter for HTTP(s) repositories.
func newHTTPGetter(url, certFile, keyFile, caFile string) (getter.Getter, error) {
	return getter.NewHTTPGetter(url, certFile, keyFile, caFile)
}

func stringsContain(list []string, s string) bool {
	for _, x := range list {
		if x == s {
			return true
		}
	}
	return false
}

func stringsDelete(list []string, s string) (res []string) {
	for _, x := range list {
		if x != s {
			res = append(res, x)
		}
	}
	return res
}

// setOwnerReference ensures the owner reference is set and returns true if it did
// not exist before. Existing references are detected based on the UID field.
func setOwnerReference(om *meta.ObjectMeta, ref meta.OwnerReference) bool {
	for i, or := range om.OwnerReferences {
		if ref.UID == or.UID {
			om.OwnerReferences[i] = ref
			return false
		}
	}
	om.OwnerReferences = append(om.OwnerReferences, ref)
	return true
}

// inCondition returns true if the ChartAssignment has a condition of the given
// type in state true.
func inCondition(as *apps.ChartAssignment, c apps.ChartAssignmentConditionType) bool {
	for _, cond := range as.Status.Conditions {
		if cond.Type == c && cond.Status == core.ConditionTrue {
			return true
		}
	}
	return false
}

// setCondition adds or updates a condition. Existing conditions are detected
// based on the Type field.
func setCondition(as *apps.ChartAssignment, t apps.ChartAssignmentConditionType, v core.ConditionStatus, msg string) {
	now := meta.Now()

	for i, c := range as.Status.Conditions {
		if c.Type != t {
			continue
		}
		// Update existing condition.
		c.LastUpdateTime = now
		if c.Status != v {
			c.LastTransitionTime = now
		}
		c.Message = msg
		c.Status = v
		as.Status.Conditions[i] = c
		return
	}
	// Condition set for the first time.
	as.Status.Conditions = append(as.Status.Conditions, apps.ChartAssignmentCondition{
		Type:               t,
		LastUpdateTime:     now,
		LastTransitionTime: now,
		Status:             v,
		Message:            msg,
	})
}

// chartPhase translates from Helm status codes to ChartAssignment phases.
func chartPhase(c release.Status_Code) apps.ChartAssignmentPhase {
	switch c {
	case release.Status_UNKNOWN:
		return apps.ChartAssignmentPhaseUnknown
	case release.Status_DEPLOYED:
		return apps.ChartAssignmentPhaseDeployed
	case release.Status_DELETING:
		return apps.ChartAssignmentPhaseDeleting
	case release.Status_DELETED:
		return apps.ChartAssignmentPhaseDeleted
	case release.Status_SUPERSEDED:
		return apps.ChartAssignmentPhaseSuperseded
	case release.Status_FAILED:
		return apps.ChartAssignmentPhaseFailed
	case release.Status_PENDING_INSTALL:
		return apps.ChartAssignmentPhaseInstall
	case release.Status_PENDING_UPGRADE:
		return apps.ChartAssignmentPhaseUpgrade
	case release.Status_PENDING_ROLLBACK:
		return apps.ChartAssignmentPhaseRollback
	default:
		return apps.ChartAssignmentPhaseUnknown
	}
}

// NewValidationWebhook returns a new webhook that validates ChartAssignments.
func NewValidationWebhook(mgr manager.Manager) (webhook.Webhook, error) {
	return builder.NewWebhookBuilder().
		Name("chartassignments.apps.cloudrobotics.com").
		Validating().
		Operations(admissionregistration.Create, admissionregistration.Update).
		FailurePolicy(admissionregistration.Fail).
		WithManager(mgr).
		ForType(&apps.ChartAssignment{}).
		Handlers(newChartAssignmentValidator(mgr.GetScheme())).
		Build()
}

// chartAssignmentValidator implements a validation webhook.
type chartAssignmentValidator struct {
	decoder runtime.Decoder
}

func newChartAssignmentValidator(sc *runtime.Scheme) *chartAssignmentValidator {
	return &chartAssignmentValidator{
		decoder: serializer.NewCodecFactory(sc).UniversalDeserializer(),
	}
}

func (v *chartAssignmentValidator) Handle(_ context.Context, req admissiontypes.Request) admissiontypes.Response {
	cur := &apps.ChartAssignment{}
	old := &apps.ChartAssignment{}

	if err := runtime.DecodeInto(v.decoder, req.AdmissionRequest.Object.Raw, cur); err != nil {
		return admission.ErrorResponse(http.StatusBadRequest, err)
	}
	if len(req.AdmissionRequest.OldObject.Raw) > 0 {
		if err := runtime.DecodeInto(v.decoder, req.AdmissionRequest.OldObject.Raw, old); err != nil {
			return admission.ErrorResponse(http.StatusBadRequest, err)
		}
	} else {
		old = nil
	}
	if err := validate(cur, old); err != nil {
		return admission.ValidationResponse(false, err.Error())
	}
	return admission.ValidationResponse(true, "")
}

func validate(cur, old *apps.ChartAssignment) error {
	if cur.Spec.ClusterName == "" {
		return fmt.Errorf("cluster name missing")
	}
	if cur.Spec.NamespaceName == "" {
		return fmt.Errorf("namespace name missing")
	}
	errs := validation.ValidateNamespaceName(cur.Spec.NamespaceName, false)
	if len(errs) > 0 {
		return fmt.Errorf("invalid namespace name: %s", strings.Join(errs, ", "))
	}
	errs = validation.ValidateClusterName(cur.Spec.ClusterName, false)
	if len(errs) > 0 {
		return fmt.Errorf("invalid cluster name: %s", strings.Join(errs, ", "))
	}
	if old != nil {
		if cur.Spec.NamespaceName != old.Spec.NamespaceName {
			return fmt.Errorf("target namespace name must not be changed")
		}
		if cur.Spec.ClusterName != old.Spec.ClusterName {
			return fmt.Errorf("target cluster name must not be changed")
		}
	}
	c := cur.Spec.Chart
	if c.Inline != "" {
		if c.Repository != "" || c.Name != "" || c.Version != "" {
			return fmt.Errorf("chart repository, name, and version must be empty for inline charts")
		}
	} else if c.Repository == "" || c.Name == "" || c.Version == "" {
		return fmt.Errorf("non-inline chart must be fully specified")
	}
	return nil
}
