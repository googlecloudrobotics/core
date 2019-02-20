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

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	admissionregistration "k8s.io/api/admissionregistration/v1beta1"
	core "k8s.io/api/core/v1"
	k8serrors "k8s.io/apimachinery/pkg/api/errors"
	"k8s.io/apimachinery/pkg/api/validation"
	meta "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/runtime"
	"k8s.io/apimachinery/pkg/runtime/serializer"
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
			kube:    mgr.GetClient(),
			helm:    hclient.NewClient(hclient.Host(tillerHost)),
			cluster: cluster,
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
	kube    kclient.Client
	helm    hclient.Interface
	cluster string // Cluster for which to handle AppAssignments.
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
		// when remvoing the finalizer. Thus, there's nothing to do.
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

func (r *Reconciler) reconcile(ctx context.Context, as *apps.ChartAssignment) (reconcile.Result, error) {
	// If we are scheduled for deletion, delete the Helm release and drop our
	// finalizer so garbage collection can continue.
	if as.DeletionTimestamp != nil {
		log.Printf("ChartAssignment %q deleted, deleting Helm release", as.Name)

		if err := r.deleteChart(as); err != nil && !strings.Contains(err.Error(), "not found") {
			return reconcile.Result{}, err
		}
		as.Finalizers = stringsDelete(as.Finalizers, finalizer)
		if err := r.kube.Update(ctx, as); err != nil {
			return reconcile.Result{}, fmt.Errorf("update failed: %s", err)
		}
		return reconcile.Result{}, nil
	}

	// If the spec did not change, only update the status of the chart release.
	if as.Generation == as.Status.ObservedGeneration {
		log.Printf("ChartAssignment %q up-to-date, updating status", as.Name)

		err := r.setChartStatus(as)
		if rev, ok := err.(errRollbackRequest); ok {
			log.Printf("ChartAssignment %q failing, rolling back", as.Name)

			if err := r.rollbackChart(as, int32(rev)); err != nil {
				return reconcile.Result{}, fmt.Errorf("rollback failed: %s", err)
			}
			// Update chart status again after rollback.
			err := r.setChartStatus(as)
			if _, ok := err.(errRollbackRequest); err != nil && ok {
				return reconcile.Result{}, fmt.Errorf("setting chart status failed: %s", err)
			}
		} else if err != nil {
			return reconcile.Result{}, fmt.Errorf("setting chart status failed: %s", err)
		}
		if err := r.kube.Status().Update(ctx, as); err != nil {
			return reconcile.Result{}, fmt.Errorf("status update failed: %s", err)
		}
		// Requeue ourselves to periodicially update the status.
		// This could be realized more cleanly by watching the underlying resources
		// in which Helm stores its state. However, since this is an implementation detail
		// rather than an official API, we stick with the safe approach.

		// Don't requeue for status updates as frequently once we are in
		// a good state.
		if as.Status.Phase == apps.ChartAssignmentPhaseDeployed {
			return reconcile.Result{Requeue: true, RequeueAfter: requeueSlow}, nil
		}
		return reconcile.Result{Requeue: true, RequeueAfter: requeueFast}, nil
	}

	log.Printf("ChartAssignment %q (version: %s) changed, updating chart release", as.Name, as.ResourceVersion)

	// Create application namespace if it doesn't exist.
	var ns core.Namespace
	err := r.kube.Get(ctx, kclient.ObjectKey{Name: as.Spec.NamespaceName}, &ns)

	if err != nil && !k8serrors.IsNotFound(err) {
		return reconcile.Result{}, fmt.Errorf("getting Namespace %q failed: %s", as.Spec.NamespaceName, err)
	}
	createNamespace := k8serrors.IsNotFound(err)
	ns.Name = as.Spec.NamespaceName

	// Add ourselves to the owners if we aren't already.
	_true := true
	setOwnerReference(&ns.ObjectMeta, meta.OwnerReference{
		APIVersion:         as.APIVersion,
		Kind:               as.Kind,
		Name:               as.Name,
		UID:                as.UID,
		BlockOwnerDeletion: &_true,
	})

	if createNamespace {
		err = r.kube.Create(ctx, &ns)
	} else {
		err = r.kube.Update(ctx, &ns)
	}
	if err != nil {
		return reconcile.Result{}, err
	}

	// Set a finalizer on the ChartAssignment so we don't get deleted before
	// we've properly deleted the associated Helm release.
	if !stringsContain(as.Finalizers, finalizer) {
		as.Finalizers = append(as.Finalizers, finalizer)
		if err := r.kube.Update(ctx, as); err != nil {
			return reconcile.Result{}, err
		}
	}

	if err := r.applyChart(as); err != nil {
		return reconcile.Result{}, err
	}

	// Update the observed generation to ensure we don't keep updating an
	// unchanged chart. This is particulary important because Helm will always
	// create a new revision not check whether there's an actual diff.
	as.Status.ObservedGeneration = as.Generation
	// Update chart status but don't apply requested rollbacks. We enqueue
	// the item again below for that.
	err = r.setChartStatus(as)
	if _, ok := err.(errRollbackRequest); err != nil && !ok {
		return reconcile.Result{}, fmt.Errorf("setting chart status failed: %s", err)
	}
	if err := r.kube.Status().Update(ctx, as); err != nil {
		return reconcile.Result{}, fmt.Errorf("status update failed: %s", err)
	}
	// Requeue in any case to update the status as the deployment progresses.
	return reconcile.Result{Requeue: true, RequeueAfter: requeueFast}, nil
}

type errRollbackRequest int

func (e errRollbackRequest) Error() string {
	return fmt.Sprintf("rollback to revision %d requested", e)
}

// setChartStatus updates the status section of the chart based on Helm's reported
// state. It returns true if a rollback should be attempted.
func (r *Reconciler) setChartStatus(as *apps.ChartAssignment) error {
	history, err := r.helm.ReleaseHistory(as.Name, hclient.WithMaxHistory(100))
	if err != nil {
		return fmt.Errorf("release history retrieval failed: %s", err)
	}
	if len(history.Releases) == 0 {
		return fmt.Errorf("no release found")
	}
	active := history.Releases[0]

	activeDesc := decodeReleaseDesc(active.Info.Description)
	// Most recent revision is what's currently deployed.
	as.Status.DeployedRevision = active.Version
	// If the current revision is a rollback, set the rollback revision. If we couldn't
	// determine it from history, it's set to 0 and will be left blank.
	if activeDesc.action == releaseActionRollback {
		as.Status.RollbackRevision = activeDesc.rollbackTo
		setCondition(as, apps.ChartAssignmentConditionRolledBack, core.ConditionTrue)
	} else {
		setCondition(as, apps.ChartAssignmentConditionRolledBack, core.ConditionFalse)
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

	as.Status.Phase = chartPhase(active.Info.Status.Code)

	switch as.Status.Phase {
	case apps.ChartAssignmentPhaseDeployed:
		setCondition(as, apps.ChartAssignmentConditionReady, core.ConditionTrue)
		setCondition(as, apps.ChartAssignmentConditionUpdated, core.ConditionTrue)

	case apps.ChartAssignmentPhaseInstall, apps.ChartAssignmentPhaseUpgrade:
		setCondition(as, apps.ChartAssignmentConditionReady, core.ConditionFalse)
		setCondition(as, apps.ChartAssignmentConditionUpdated, core.ConditionTrue)

	case apps.ChartAssignmentPhaseFailed:
		setCondition(as, apps.ChartAssignmentConditionReady, core.ConditionFalse)
		setCondition(as, apps.ChartAssignmentConditionUpdated, core.ConditionTrue)

		// Find the last successful release that we may want to rollback to.
		var rollbackRevision int32
		for _, r := range history.Releases[1:] {
			if d := decodeReleaseDesc(r.Info.Description); !d.failed {
				rollbackRevision = r.Version
				break
			}
		}
		// Trigger a rollback if the current revision isn't already one.
		a := decodeReleaseAction(active.Info.Description)
		if a != releaseActionRollback && a != releaseActionUnknown && rollbackRevision > 0 {
			return errRollbackRequest(rollbackRevision)
		}
		return nil

	default:
		setCondition(as, apps.ChartAssignmentConditionReady, core.ConditionFalse)
		setCondition(as, apps.ChartAssignmentConditionUpdated, core.ConditionFalse)
	}
	return nil
}

// rollbackChart will rollback the chart to the previous revision if it exists
// and wasn't a rollback itself.
func (r *Reconciler) rollbackChart(as *apps.ChartAssignment, rev int32) error {
	_, err := r.helm.RollbackRelease(as.Name,
		hclient.RollbackForce(true),
		hclient.RollbackTimeout(tillerTimeout),
		hclient.RollbackVersion(rev),
	)
	return err
}

// applyChart installs or updates the chart.
func (r *Reconciler) applyChart(as *apps.ChartAssignment) error {
	var (
		cspec   = as.Spec.Chart
		archive io.Reader
		err     error
	)
	if cspec.Inline != "" {
		archive = base64.NewDecoder(base64.StdEncoding, strings.NewReader(cspec.Inline))
	} else {
		archive, err = r.fetchChartTar(cspec.Repository, cspec.Name, cspec.Version)
		if err != nil {
			return fmt.Errorf("retrieving chart failed: %s", err)
		}
	}
	c, err := chartutil.LoadArchive(archive)
	if err != nil {
		return fmt.Errorf("loading chart failed: %s", err)
	}

	// Ensure charts in requirements.yaml are actually in packaged in.
	if req, err := chartutil.LoadRequirements(c); err == nil {
		if err := renderutil.CheckDependencies(c, req); err != nil {
			return fmt.Errorf("chart dependency error: %s", err)
		}
	} else if err != chartutil.ErrRequirementsNotFound {
		return fmt.Errorf("cannot load requirements: %v", err)
	}

	// Build the full set of values including the default ones. Even though
	// they are part of the chart, they are ignored if we don't provide
	// them explicitly.
	vals, err := chartutil.ReadValues([]byte(c.Values.Raw))
	if err != nil {
		return fmt.Errorf("reading chart values failed: %s", err)
	}
	vals.MergeInto(chartutil.Values(as.Spec.Chart.Values)) // ChartAssignment values.

	valsRaw, err := vals.YAML()
	if err != nil {
		return fmt.Errorf("encoding values failed: %s", err)
	}

	// Install if the release doesn't exist yet, update otherwise.
	_, err = r.helm.ReleaseStatus(as.Name)
	if err != nil {
		if !strings.Contains(err.Error(), "not found") {
			return fmt.Errorf("getting release status failed: %s", err)
		}
		// First instance of this release for the namespace.
		_, err = r.helm.InstallReleaseFromChart(c, as.Spec.NamespaceName,
			hclient.ReleaseName(as.Name),
			hclient.InstallTimeout(tillerTimeout),
			hclient.ValueOverrides([]byte(valsRaw)),
		)
		if err != nil {
			return fmt.Errorf("chart installation failed: %s", err)
		}
		return nil
	}
	_, err = r.helm.UpdateReleaseFromChart(as.Name, c,
		hclient.UpgradeForce(true),
		hclient.UpgradeTimeout(tillerTimeout),
		hclient.UpdateValueOverrides([]byte(valsRaw)),
	)
	if err != nil {
		return fmt.Errorf("chart update failed: %s", err)
	}
	return nil
}

func (r *Reconciler) deleteChart(as *apps.ChartAssignment) error {
	_, err := r.helm.DeleteRelease(as.Name,
		hclient.DeletePurge(true),
		hclient.DeleteTimeout(tillerTimeout),
	)
	if err != nil {
		return fmt.Errorf("deleting chart failed: %s", err)
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
	if strings.Contains(s, "delete") {
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

// setOwnerReference adds or updates an owner reference. Existing references
// are detected based on the UID field.
func setOwnerReference(om *meta.ObjectMeta, ref meta.OwnerReference) {
	for i, or := range om.OwnerReferences {
		if ref.UID == or.UID {
			om.OwnerReferences[i] = ref
			return
		}
	}
	om.OwnerReferences = append(om.OwnerReferences, ref)
}

// setCondition adds or updates a condition. Existing conditions are detected
// based on the Type field.
func setCondition(as *apps.ChartAssignment, t apps.ChartAssignmentConditionType, v core.ConditionStatus) {
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
