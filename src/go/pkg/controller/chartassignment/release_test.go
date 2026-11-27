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

package chartassignment

import (
	"archive/tar"
	"bytes"
	"compress/gzip"
	"encoding/base64"
	"errors"
	"fmt"
	"net/http"
	"net/http/httptest"
	"strings"
	"sync/atomic"
	"testing"
	"testing/synctest"
	"time"

	"github.com/golang/mock/gomock"
	"github.com/google/go-cmp/cmp"
	"go.opentelemetry.io/otel/trace"
	"helm.sh/helm/v3/pkg/chartutil"
	"helm.sh/helm/v3/pkg/repo"
	k8serrors "k8s.io/apimachinery/pkg/api/errors"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/apis/meta/v1/unstructured"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/client-go/tools/record"

	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"github.com/googlecloudrobotics/core/src/go/pkg/kubetest"
)

const (
	ChartName = "testchart"
)

func verifyValues(t *testing.T, haveValues chartutil.Values, wantValues chartutil.Values) {
	t.Helper()
	have, err := haveValues.YAML()
	if err != nil {
		t.Fatal(err)
	}
	want, err := wantValues.YAML()
	if err != nil {
		t.Fatal(err)
	}
	if want != have {
		t.Fatalf("config values do not match: want\n%s\n\ngot\n%s\n", want, have)
	}
}

func Test_loadAndExpandChart_mergesValues(t *testing.T) {
	var as apps.ChartAssignment
	unmarshalYAML(t, &as, `
metadata:
  name: test-assignment-1
spec:
  chart:
    values:
      bar1: 4
      bar2:
        baz2: test
	`)
	as.Spec.Chart.Inline = kubetest.BuildInlineChart(t, ChartName, `
apiVersion: v1
kind: ConfigMap
metadata:
  name: values
data:
  bar1: "{{ .Values.bar1 }}"
  baz1: "{{ .Values.foo1.baz1 }}"
  baz2: "{{ .Values.bar2.baz2 }}"
`, `
foo1:
  baz1: "hello"
bar1: 3`)
	resources, _, err := loadAndExpandChart(&as)
	if err != nil {
		t.Fatal(err)
	}
	if len(resources) != 1 {
		t.Fatalf("want 1 resource, got %d", len(resources))
	}
	got, _, _ := unstructured.NestedStringMap(resources[0].Object, "data")
	want := map[string]string{"bar1": "4", "baz1": "hello", "baz2": "test"}
	if diff := cmp.Diff(want, got); diff != "" {
		t.Errorf("rendered values mismatch (-want +got):\n%s", diff)
	}
}

func Test_loadChartWithoutTemplates_returnsZeroManifests(t *testing.T) {
	var as apps.ChartAssignment
	unmarshalYAML(t, &as, `
metadata:
  name: test-assignment-1
spec:
  chart:
    values:
	`)
	as.Spec.Chart.Inline = kubetest.BuildInlineChart(t, ChartName /*template=*/, "", `foo: 1`)
	resources, _, err := loadAndExpandChart(&as)
	if err != nil {
		t.Fatal(err)
	}
	if len(resources) > 0 {
		t.Errorf("Expected no resources, got %d", len(resources))
	}
}

func Test_updateSynk_callsApply(t *testing.T) {
	ctrl := gomock.NewController(t)
	defer ctrl.Finish()

	var as apps.ChartAssignment
	unmarshalYAML(t, &as, `
metadata:
  name: test-assignment-1
spec:
  chart:
    values:
	`)
	as.Spec.Chart.Inline = kubetest.BuildInlineChart(t, ChartName /*template=*/, "", `foo: 1`)

	mockSynk := NewMockInterface(ctrl)
	r := &release{
		ctx:      t.Context(),
		synk:     mockSynk,
		recorder: &record.FakeRecorder{},
	}

	rs := &apps.ResourceSet{}
	mockSynk.EXPECT().Apply(gomock.Any(), "test-assignment-1", gomock.Any(), gomock.Any()).Return(rs, nil).Times(1)

	// First apply, the chart should be installed.
	r.update(&as)
}

func Test_deleteSynk_callsDelete(t *testing.T) {
	ctrl := gomock.NewController(t)
	defer ctrl.Finish()

	var as apps.ChartAssignment
	unmarshalYAML(t, &as, `
metadata:
  name: test-assignment-1
spec:
  chart:
    values:
	`)
	as.Spec.Chart.Inline = kubetest.BuildInlineChart(t, ChartName /*template=*/, "", `foo: 1`)

	mockSynk := NewMockInterface(ctrl)
	r := &release{
		ctx:      t.Context(),
		synk:     mockSynk,
		recorder: &record.FakeRecorder{},
	}

	mockSynk.EXPECT().Delete(gomock.Any(), "test-assignment-1").Return(nil).Times(1)

	// First apply, the chart should be installed.
	r.delete(&as)
}

func pollStatus(t *testing.T, rs *releases, name string, check func(releaseStatus) bool) {
	t.Helper()
	synctest.Wait()
	status, ok := rs.status(name)
	if !ok || !check(status) {
		t.Fatalf("Status check failed. Current status: %+v", status)
	}
}

func startWithRetry(t *testing.T, r *release, f func()) {
	t.Helper()
	for i := 0; i < 100; i++ {
		if r.start(f) {
			return
		}
		synctest.Wait()
	}
	t.Fatal("Timeout waiting for release worker to start function")
}

func ensureUpdatedWithRetry(t *testing.T, rs *releases, as *apps.ChartAssignment) {
	t.Helper()
	for i := 0; i < 100; i++ {
		if rs.ensureUpdated(as) {
			return
		}
		synctest.Wait()
	}
	t.Fatal("Timeout waiting for ensureUpdated to start")
}

func TestReleases_EnsureUpdated(t *testing.T) {
	synctest.Test(t, func(t *testing.T) {
		ctrl := gomock.NewController(t)
		defer ctrl.Finish()

		mockSynk := NewMockInterface(ctrl)
		recorder := &record.FakeRecorder{
			Events: make(chan string, 10),
		}
		rs := &releases{
			ctx:      t.Context(),
			recorder: recorder,
			m:        map[string]*release{},
			synk:     mockSynk,
		}

		as := &apps.ChartAssignment{
			ObjectMeta: metav1.ObjectMeta{
				Name:       "my-release",
				Generation: 1,
			},
			Spec: apps.ChartAssignmentSpec{
				NamespaceName: "ns1",
				Chart: apps.AssignedChart{
					Inline: kubetest.BuildInlineChart(t, ChartName, "", "foo: 1"),
				},
			},
		}

		// 1. Initial update
		mockSynk.EXPECT().Apply(gomock.Any(), "my-release", gomock.Any(), gomock.Any()).Return(&apps.ResourceSet{}, nil).Times(1)

		ensureUpdatedWithRetry(t, rs, as)

		pollStatus(t, rs, "my-release", func(status releaseStatus) bool {
			return status.phase == apps.ChartAssignmentPhaseSettled
		})

		// 2. Update with same generation should skip
		ensureUpdatedWithRetry(t, rs, as)

		// 3. Update with new generation should trigger update
		as.Generation = 2
		mockSynk.EXPECT().Apply(gomock.Any(), "my-release", gomock.Any(), gomock.Any()).Return(&apps.ResourceSet{}, nil).Times(1)

		ensureUpdatedWithRetry(t, rs, as)

		pollStatus(t, rs, "my-release", func(status releaseStatus) bool {
			return status.phase == apps.ChartAssignmentPhaseSettled && rs.m["my-release"].generation() == 2
		})
	})
}

func TestReleases_EnsureUpdated_Busy(t *testing.T) {
	synctest.Test(t, func(t *testing.T) {
		ctrl := gomock.NewController(t)
		defer ctrl.Finish()

		mockSynk := NewMockInterface(ctrl)
		recorder := &record.FakeRecorder{
			Events: make(chan string, 10),
		}
		rs := &releases{
			ctx:      t.Context(),
			recorder: recorder,
			m:        map[string]*release{},
			synk:     mockSynk,
		}

		as := &apps.ChartAssignment{
			ObjectMeta: metav1.ObjectMeta{
				Name:       "my-release",
				Generation: 1,
			},
			Spec: apps.ChartAssignmentSpec{
				NamespaceName: "ns1",
				Chart: apps.AssignedChart{
					Inline: kubetest.BuildInlineChart(t, ChartName, "", "foo: 1"),
				},
			},
		}

		r := rs.add(as.Name)

		blockChan := make(chan struct{})
		unblockChan := make(chan struct{})

		startWithRetry(t, r, func() {
			close(blockChan)
			<-unblockChan
		})

		<-blockChan

		// Now worker is busy. ensureUpdated should return false without clobbering status.
		started := rs.ensureUpdated(as)
		if started {
			t.Error("Expected ensureUpdated to return false because worker is busy")
		}
		if status, _ := rs.status(as.Name); status.phase != apps.ChartAssignmentPhaseAccepted {
			t.Errorf("Expected phase to remain Accepted while worker is busy, got %q", status.phase)
		}

		close(unblockChan)
	})
}

func TestReleases_EnsureUpdated_TransientErrorPreservesErrOnRetry(t *testing.T) {
	transientErr := fmt.Errorf("temporary network error")
	r := &release{
		ctx:    t.Context(),
		name:   "my-release",
		actorc: make(chan func(), 1),
		gen:    1,
		status: releaseStatus{
			phase: apps.ChartAssignmentPhaseLoadingChart,
			err:   transientErr,
			retry: true,
		},
	}
	rs := &releases{
		ctx: t.Context(),
		m:   map[string]*release{"my-release": r},
	}
	as := &apps.ChartAssignment{
		ObjectMeta: metav1.ObjectMeta{
			Name:       "my-release",
			Generation: 1,
		},
	}

	// Retrying the same generation should preserve the previous error so setStatus can report it.
	if !rs.ensureUpdated(as) {
		t.Fatal("Expected ensureUpdated to start retry")
	}
	status, ok := rs.status(as.Name)
	if !ok {
		t.Fatal("Expected release status to exist")
	}
	if status.phase != apps.ChartAssignmentPhaseLoadingChart {
		t.Errorf("Expected phase LoadingChart, got %q", status.phase)
	}
	if status.err != transientErr {
		t.Errorf("Expected error %v to be preserved on retry, got %v", transientErr, status.err)
	}
	if status.retry {
		t.Error("Expected retry flag to be cleared once retry started")
	}

	// Drain queued work and verify a new generation clears the old error.
	<-r.actorc
	as.Generation = 2
	if !rs.ensureUpdated(as) {
		t.Fatal("Expected ensureUpdated to start update for new generation")
	}
	status, _ = rs.status(as.Name)
	if status.err != nil {
		t.Errorf("Expected error to be cleared for new generation, got %v", status.err)
	}
	if r.generation() != 2 {
		t.Errorf("Expected generation 2, got %d", r.generation())
	}
}

func TestReleases_EnsureUpdated_TransientErrorRetry(t *testing.T) {
	synctest.Test(t, func(t *testing.T) {
		ctrl := gomock.NewController(t)
		defer ctrl.Finish()

		mockSynk := NewMockInterface(ctrl)
		recorder := &record.FakeRecorder{
			Events: make(chan string, 10),
		}
		rs := &releases{
			ctx:      t.Context(),
			recorder: recorder,
			m:        map[string]*release{},
			synk:     mockSynk,
		}

		as := &apps.ChartAssignment{
			ObjectMeta: metav1.ObjectMeta{
				Name:       "my-release",
				Generation: 1,
			},
			Spec: apps.ChartAssignmentSpec{
				NamespaceName: "ns1",
				Chart: apps.AssignedChart{
					Inline: kubetest.BuildInlineChart(t, ChartName, "", "foo: 1"),
				},
			},
		}

		// 1. First attempt fails with transient error (Conflict)
		transientErr := k8serrors.NewConflict(schema.GroupResource{Group: "apps", Resource: "chartassignments"}, "my-release", fmt.Errorf("conflict"))
		mockSynk.EXPECT().Apply(gomock.Any(), "my-release", gomock.Any(), gomock.Any()).Return(nil, transientErr).Times(1)

		ensureUpdatedWithRetry(t, rs, as)

		pollStatus(t, rs, "my-release", func(s releaseStatus) bool {
			return s.phase == apps.ChartAssignmentPhaseUpdating && s.err != nil && s.retry == true
		})

		// 2. Second attempt with same generation should NOT be skipped because retry is true.
		mockSynk.EXPECT().Apply(gomock.Any(), "my-release", gomock.Any(), gomock.Any()).Return(&apps.ResourceSet{}, nil).Times(1)

		ensureUpdatedWithRetry(t, rs, as)

		pollStatus(t, rs, "my-release", func(s releaseStatus) bool {
			return s.phase == apps.ChartAssignmentPhaseSettled && s.err == nil && s.retry == false
		})
	})
}

func TestReleases_EnsureUpdated_PermanentErrorNoRetry(t *testing.T) {
	synctest.Test(t, func(t *testing.T) {
		ctrl := gomock.NewController(t)
		defer ctrl.Finish()

		mockSynk := NewMockInterface(ctrl)
		recorder := &record.FakeRecorder{
			Events: make(chan string, 10),
		}
		rs := &releases{
			ctx:      t.Context(),
			recorder: recorder,
			m:        map[string]*release{},
			synk:     mockSynk,
		}

		as := &apps.ChartAssignment{
			ObjectMeta: metav1.ObjectMeta{
				Name:       "my-release",
				Generation: 1,
			},
			Spec: apps.ChartAssignmentSpec{
				NamespaceName: "ns1",
				Chart: apps.AssignedChart{
					Inline: kubetest.BuildInlineChart(t, ChartName, "", "foo: 1"),
				},
			},
		}

		// 1. First attempt fails with permanent error
		permanentErr := fmt.Errorf("permanent error")
		mockSynk.EXPECT().Apply(gomock.Any(), "my-release", gomock.Any(), gomock.Any()).Return(nil, permanentErr).Times(1)

		ensureUpdatedWithRetry(t, rs, as)

		pollStatus(t, rs, "my-release", func(s releaseStatus) bool {
			return s.phase == apps.ChartAssignmentPhaseFailed && s.err != nil && s.retry == false
		})

		// 2. Second attempt with same generation should be skipped.
		ensureUpdatedWithRetry(t, rs, as)
	})
}

func TestExtractParentSpanContext(t *testing.T) {
	t.Run("W3C traceparent", func(t *testing.T) {
		as := &apps.ChartAssignment{
			ObjectMeta: metav1.ObjectMeta{
				Name: "my-release",
				Annotations: map[string]string{
					"traceparent": "00-4bf92f3577b34da6a3ce929d0e0e4736-00f067aa0ba902b7-01",
				},
			},
		}
		ctx := extractParentSpanContext(t.Context(), as)
		sc := trace.SpanContextFromContext(ctx)
		if !sc.IsValid() || !sc.IsRemote() || !sc.IsSampled() {
			t.Fatalf("Expected valid remote sampled SpanContext, got %+v", sc)
		}
		if got := sc.TraceID().String(); got != "4bf92f3577b34da6a3ce929d0e0e4736" {
			t.Errorf("Expected TraceID = 4bf92f3577b34da6a3ce929d0e0e4736, got %q", got)
		}
		if got := sc.SpanID().String(); got != "00f067aa0ba902b7" {
			t.Errorf("Expected SpanID = 00f067aa0ba902b7, got %q", got)
		}
	})

	t.Run("legacy cloudrobotics.com/trace-id fallback", func(t *testing.T) {
		as := &apps.ChartAssignment{
			ObjectMeta: metav1.ObjectMeta{
				Name: "my-release",
				Annotations: map[string]string{
					"cloudrobotics.com/trace-id": "4bf92f3577b34da6a3ce929d0e0e4736",
				},
			},
		}
		ctx := extractParentSpanContext(t.Context(), as)
		sc := trace.SpanContextFromContext(ctx)
		if !sc.IsValid() || !sc.IsRemote() || !sc.IsSampled() {
			t.Fatalf("Expected valid remote sampled SpanContext from legacy annotation, got %+v", sc)
		}
		if got := sc.TraceID().String(); got != "4bf92f3577b34da6a3ce929d0e0e4736" {
			t.Errorf("Expected TraceID = 4bf92f3577b34da6a3ce929d0e0e4736, got %q", got)
		}
	})
}

func Test_httpGetter(t *testing.T) {
	g, err := newHTTPGetter()
	if err != nil {
		t.Fatalf("newHTTPGetter() error = %v", err)
	}

	t.Run("200 OK", func(t *testing.T) {
		srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			w.Write([]byte("chart-bytes"))
		}))
		defer srv.Close()

		buf, err := g.Get(srv.URL)
		if err != nil {
			t.Fatalf("Get() error = %v", err)
		}
		if got := buf.String(); got != "chart-bytes" {
			t.Errorf("Get() = %q, want %q", got, "chart-bytes")
		}
	})

	t.Run("preserves gzip body when Content-Encoding is gzip", func(t *testing.T) {
		var gzBuf bytes.Buffer
		zw := gzip.NewWriter(&gzBuf)
		zw.Write([]byte("tar-content"))
		zw.Close()
		rawGzip := gzBuf.Bytes()

		srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			w.Header().Set("Content-Encoding", "gzip")
			w.Write(rawGzip)
		}))
		defer srv.Close()

		buf, err := g.Get(srv.URL)
		if err != nil {
			t.Fatalf("Get() error = %v", err)
		}
		if !bytes.Equal(buf.Bytes(), rawGzip) {
			t.Errorf("Get() modified gzip payload; got %d bytes, want %d raw gzip bytes", buf.Len(), len(rawGzip))
		}
	})

	statusCases := []struct {
		code          int
		wantTransient bool
	}{
		{http.StatusBadRequest, false},
		{http.StatusUnauthorized, false},
		{http.StatusForbidden, false},
		{http.StatusNotFound, false},
		{http.StatusGone, false},
		{http.StatusRequestTimeout, true},
		{http.StatusTooManyRequests, true},
		{http.StatusInternalServerError, true},
		{http.StatusNotImplemented, false},
		{http.StatusBadGateway, true},
		{http.StatusServiceUnavailable, true},
		{http.StatusGatewayTimeout, true},
		{http.StatusHTTPVersionNotSupported, false},
		{http.StatusLoopDetected, false},
	}
	for _, tc := range statusCases {
		t.Run(fmt.Sprintf("status_%d", tc.code), func(t *testing.T) {
			srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
				http.Error(w, "fail", tc.code)
			}))
			defer srv.Close()

			_, err := g.Get(srv.URL)
			if err == nil {
				t.Fatalf("Get() succeeded, want error for status %d", tc.code)
			}
			var statusErr *httpStatusError
			if !errors.As(err, &statusErr) {
				t.Fatalf("Get() error = %T (%v), want *httpStatusError", err, err)
			}
			if statusErr.statusCode != tc.code {
				t.Errorf("statusErr.statusCode = %d, want %d", statusErr.statusCode, tc.code)
			}
			if got := statusErr.isTransient(); got != tc.wantTransient {
				t.Errorf("statusErr.isTransient() = %v, want %v", got, tc.wantTransient)
			}
		})
	}
}

func buildTestChartTarball(t *testing.T, name string, files map[string]string) []byte {
	t.Helper()
	var buf bytes.Buffer
	zw := gzip.NewWriter(&buf)
	tw := tar.NewWriter(zw)
	for path, content := range files {
		if err := tw.WriteHeader(&tar.Header{
			Name: name + "/" + path,
			Mode: 0644,
			Size: int64(len(content)),
		}); err != nil {
			t.Fatalf("WriteHeader(%s): %v", path, err)
		}
		if _, err := tw.Write([]byte(content)); err != nil {
			t.Fatalf("Write(%s): %v", path, err)
		}
	}
	if err := tw.Close(); err != nil {
		t.Fatalf("tar Close: %v", err)
	}
	if err := zw.Close(); err != nil {
		t.Fatalf("gzip Close: %v", err)
	}
	return buf.Bytes()
}

func Test_loadAndExpandChart_RemoteRepo(t *testing.T) {
	chartTgz, err := base64.StdEncoding.DecodeString(kubetest.BuildInlineChart(t, "kyverno", `
apiVersion: v1
kind: ConfigMap
metadata:
  name: kyverno-cfg
data:
  key: val
`, ""))
	if err != nil {
		t.Fatalf("decode test chart: %v", err)
	}

	const indexYAML = `apiVersion: v1
entries:
  kyverno:
    - name: kyverno
      version: 3.2.1
      urls:
        - charts/kyverno-3.2.1.tgz
    - name: kyverno
      version: 3.2.2
      urls:
        - charts/missing-404.tgz
    - name: kyverno
      version: 3.2.3
      urls:
        - charts/transient-503.tgz
    - name: kyverno
      version: 3.2.4
      urls:
        - charts/gzipped-3.2.4.tgz
  no-urls:
    - name: no-urls
      version: 1.0.0
      urls: []
`

	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		switch r.URL.Path {
		case "/index.yaml":
			w.Write([]byte(indexYAML))
		case "/charts/kyverno-3.2.1.tgz":
			w.Write(chartTgz)
		case "/charts/missing-404.tgz":
			http.Error(w, "not found", http.StatusNotFound)
		case "/charts/transient-503.tgz":
			http.Error(w, "unavailable", http.StatusServiceUnavailable)
		case "/charts/gzipped-3.2.4.tgz":
			w.Header().Set("Content-Encoding", "gzip")
			w.Write(chartTgz)
		case "/status-404/index.yaml":
			http.Error(w, "not found", http.StatusNotFound)
		case "/status-403/index.yaml":
			http.Error(w, "forbidden", http.StatusForbidden)
		case "/status-408/index.yaml":
			http.Error(w, "timeout", http.StatusRequestTimeout)
		case "/status-429/index.yaml":
			http.Error(w, "rate limited", http.StatusTooManyRequests)
		case "/status-503/index.yaml":
			http.Error(w, "unavailable", http.StatusServiceUnavailable)
		default:
			http.NotFound(w, r)
		}
	}))
	defer srv.Close()

	tests := []struct {
		name            string
		repoURL         string
		chartName       string
		version         string
		wantErr         bool
		wantRetry       bool
		wantErrContains string
		wantErrTarget   error
	}{
		{
			name:      "valid chart and version",
			repoURL:   srv.URL,
			chartName: "kyverno",
			version:   "3.2.1",
		},
		{
			name:      "valid chart served with Content-Encoding gzip",
			repoURL:   srv.URL,
			chartName: "kyverno",
			version:   "3.2.4",
		},
		{
			name:            "missing chart version is permanent",
			repoURL:         srv.URL,
			chartName:       "kyverno",
			version:         "1.12.1",
			wantErr:         true,
			wantRetry:       false,
			wantErrContains: `chart "kyverno" version "1.12.1" not found in`,
		},
		{
			name:          "missing chart name is permanent",
			repoURL:       srv.URL,
			chartName:     "nonexistent",
			version:       "1.0.0",
			wantErr:       true,
			wantRetry:     false,
			wantErrTarget: repo.ErrNoChartName,
		},
		{
			name:            "invalid semver version is permanent",
			repoURL:         srv.URL,
			chartName:       "kyverno",
			version:         "not-a-semver",
			wantErr:         true,
			wantRetry:       false,
			wantErrContains: `chart "kyverno" version "not-a-semver" not found in`,
		},
		{
			name:            "chart with empty urls is permanent",
			repoURL:         srv.URL,
			chartName:       "no-urls",
			version:         "1.0.0",
			wantErr:         true,
			wantRetry:       false,
			wantErrContains: "has no downloadable URLs",
		},
		{
			name:      "invalid repo URL is permanent",
			repoURL:   "://bad-url",
			chartName: "kyverno",
			version:   "3.2.1",
			wantErr:   true,
			wantRetry: false,
		},
		{
			name:      "unsupported repo scheme is permanent",
			repoURL:   "ftp://example.com/charts",
			chartName: "kyverno",
			version:   "3.2.1",
			wantErr:   true,
			wantRetry: false,
		},
		{
			name:            "index.yaml 404 is permanent",
			repoURL:         srv.URL + "/status-404",
			chartName:       "kyverno",
			version:         "3.2.1",
			wantErr:         true,
			wantRetry:       false,
			wantErrContains: "404 Not Found",
		},
		{
			name:            "index.yaml 403 is permanent",
			repoURL:         srv.URL + "/status-403",
			chartName:       "kyverno",
			version:         "3.2.1",
			wantErr:         true,
			wantRetry:       false,
			wantErrContains: "403 Forbidden",
		},
		{
			name:            "index.yaml 408 is transient",
			repoURL:         srv.URL + "/status-408",
			chartName:       "kyverno",
			version:         "3.2.1",
			wantErr:         true,
			wantRetry:       true,
			wantErrContains: "408 Request Timeout",
		},
		{
			name:            "index.yaml 429 is transient",
			repoURL:         srv.URL + "/status-429",
			chartName:       "kyverno",
			version:         "3.2.1",
			wantErr:         true,
			wantRetry:       true,
			wantErrContains: "429 Too Many Requests",
		},
		{
			name:            "index.yaml 503 is transient",
			repoURL:         srv.URL + "/status-503",
			chartName:       "kyverno",
			version:         "3.2.1",
			wantErr:         true,
			wantRetry:       true,
			wantErrContains: "503 Service Unavailable",
		},
		{
			name:            "chart tarball 404 is permanent",
			repoURL:         srv.URL,
			chartName:       "kyverno",
			version:         "3.2.2",
			wantErr:         true,
			wantRetry:       false,
			wantErrContains: "404 Not Found",
		},
		{
			name:            "chart tarball 503 is transient",
			repoURL:         srv.URL,
			chartName:       "kyverno",
			version:         "3.2.3",
			wantErr:         true,
			wantRetry:       true,
			wantErrContains: "503 Service Unavailable",
		},
	}

	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			as := &apps.ChartAssignment{
				ObjectMeta: metav1.ObjectMeta{Name: "test-assignment"},
				Spec: apps.ChartAssignmentSpec{
					NamespaceName: "ns1",
					Chart: apps.AssignedChart{
						Repository: tc.repoURL,
						Name:       tc.chartName,
						Version:    tc.version,
					},
				},
			}
			resources, retry, err := loadAndExpandChart(as)
			if tc.wantErr {
				if err == nil {
					t.Fatalf("loadAndExpandChart() succeeded, want error")
				}
				if retry != tc.wantRetry {
					t.Errorf("loadAndExpandChart() retry = %v, want %v (err: %v)", retry, tc.wantRetry, err)
				}
				if tc.wantErrContains != "" && !strings.Contains(err.Error(), tc.wantErrContains) {
					t.Errorf("loadAndExpandChart() err = %q, want substring %q", err.Error(), tc.wantErrContains)
				}
				if tc.wantErrTarget != nil && !errors.Is(err, tc.wantErrTarget) {
					t.Errorf("loadAndExpandChart() err = %v, want errors.Is(_, %v)", err, tc.wantErrTarget)
				}
				return
			}
			if err != nil {
				t.Fatalf("loadAndExpandChart() unexpected error: %v", err)
			}
			if len(resources) != 1 {
				t.Errorf("loadAndExpandChart() returned %d resources, want 1", len(resources))
			}
		})
	}
}

func Test_loadAndExpandChart_InvalidInlineArchive(t *testing.T) {
	as := &apps.ChartAssignment{
		ObjectMeta: metav1.ObjectMeta{Name: "test-assignment"},
		Spec: apps.ChartAssignmentSpec{
			NamespaceName: "ns1",
			Chart: apps.AssignedChart{
				Inline: base64.StdEncoding.EncodeToString([]byte("not-a-valid-gzip-archive")),
			},
		},
	}
	_, retry, err := loadAndExpandChart(as)
	if err == nil {
		t.Fatal("Expected error for invalid inline archive")
	}
	if retry {
		t.Errorf("Expected retry=false for invalid inline archive, got true (err: %v)", err)
	}
}

func Test_loadAndExpandChart_InvalidValuesSchema(t *testing.T) {
	tgz := buildTestChartTarball(t, "schemachart", map[string]string{
		"Chart.yaml":         `apiVersion: v2` + "\n" + `name: schemachart` + "\n" + `version: 0.1.0`,
		"values.yaml":        `replicas: 1`,
		"values.schema.json": `{"type":"object","properties":{"replicas":{"type":"integer"}}}`,
		"templates/cm.yaml":  "",
	})
	as := &apps.ChartAssignment{
		ObjectMeta: metav1.ObjectMeta{Name: "test-assignment"},
		Spec: apps.ChartAssignmentSpec{
			NamespaceName: "ns1",
			Chart: apps.AssignedChart{
				Inline: base64.StdEncoding.EncodeToString(tgz),
				Values: apps.ConfigValues{"replicas": "not-an-int"},
			},
		},
	}
	_, retry, err := loadAndExpandChart(as)
	if err == nil {
		t.Fatal("Expected error for values violating values.schema.json")
	}
	if retry {
		t.Errorf("Expected retry=false for values schema validation error, got true (err: %v)", err)
	}
}

func TestReleases_EnsureUpdated_PermanentChartVersionError(t *testing.T) {
	chartTgz, err := base64.StdEncoding.DecodeString(kubetest.BuildInlineChart(t, "kyverno", "", ""))
	if err != nil {
		t.Fatalf("decode test chart: %v", err)
	}
	const indexYAML = `apiVersion: v1
entries:
  kyverno:
    - name: kyverno
      version: 3.2.1
      urls:
        - charts/kyverno-3.2.1.tgz
`
	var indexFetches atomic.Int32
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		switch r.URL.Path {
		case "/index.yaml":
			indexFetches.Add(1)
			w.Write([]byte(indexYAML))
		case "/charts/kyverno-3.2.1.tgz":
			w.Write(chartTgz)
		default:
			http.NotFound(w, r)
		}
	}))
	defer srv.Close()

	ctrl := gomock.NewController(t)
	defer ctrl.Finish()

	mockSynk := NewMockInterface(ctrl)
	rs := &releases{
		ctx:      t.Context(),
		recorder: &record.FakeRecorder{Events: make(chan string, 10)},
		m:        map[string]*release{},
		synk:     mockSynk,
	}

	as := &apps.ChartAssignment{
		ObjectMeta: metav1.ObjectMeta{
			Name:       "my-release",
			Generation: 1,
		},
		Spec: apps.ChartAssignmentSpec{
			NamespaceName: "ns1",
			Chart: apps.AssignedChart{
				Repository: srv.URL,
				Name:       "kyverno",
				Version:    "1.12.1",
			},
		},
	}

	waitStatus := func(check func(releaseStatus) bool) releaseStatus {
		t.Helper()
		deadline := time.Now().Add(5 * time.Second)
		for time.Now().Before(deadline) {
			if status, ok := rs.status(as.Name); ok && check(status) {
				return status
			}
			time.Sleep(10 * time.Millisecond)
		}
		status, _ := rs.status(as.Name)
		t.Fatalf("Timed out waiting for status; last status: %+v", status)
		return releaseStatus{}
	}

	// Start update with bad version.
	for !rs.ensureUpdated(as) {
		time.Sleep(10 * time.Millisecond)
	}
	status := waitStatus(func(s releaseStatus) bool {
		return s.phase == apps.ChartAssignmentPhaseFailed
	})
	if status.retry {
		t.Error("Expected retry=false for missing chart version")
	}
	if status.err == nil || !strings.Contains(status.err.Error(), `version "1.12.1" not found`) {
		t.Errorf("Unexpected error: %v", status.err)
	}
	if got := indexFetches.Load(); got != 1 {
		t.Errorf("Expected 1 index fetch, got %d", got)
	}

	// Calling ensureUpdated again with the same generation must not retry.
	if !rs.ensureUpdated(as) {
		t.Fatal("Expected ensureUpdated to return true for already-failed generation")
	}
	if got := indexFetches.Load(); got != 1 {
		t.Errorf("Expected no additional index fetches on same generation, got %d", got)
	}

	// Updating to a valid version on the next generation should recover and settle.
	as.Generation = 2
	as.Spec.Chart.Version = "3.2.1"
	mockSynk.EXPECT().Apply(gomock.Any(), "my-release", gomock.Any(), gomock.Any()).Return(&apps.ResourceSet{}, nil).Times(1)
	for !rs.ensureUpdated(as) {
		time.Sleep(10 * time.Millisecond)
	}
	status = waitStatus(func(s releaseStatus) bool {
		return s.phase == apps.ChartAssignmentPhaseSettled
	})
	if status.err != nil {
		t.Errorf("Expected nil error after recovery, got %v", status.err)
	}
}
