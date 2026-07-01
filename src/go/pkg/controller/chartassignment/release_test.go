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
	"fmt"
	"testing"
	"testing/synctest"

	"github.com/golang/mock/gomock"
	apps "github.com/googlecloudrobotics/core/src/go/pkg/apis/apps/v1alpha1"
	"github.com/googlecloudrobotics/core/src/go/pkg/kubetest"
	k8serrors "k8s.io/apimachinery/pkg/api/errors"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/runtime/schema"
	"k8s.io/client-go/tools/record"
	"k8s.io/helm/pkg/chartutil"
)

const (
	ChartName = "testchart"
)

func verifyValues(t *testing.T, have string, wantValues chartutil.Values) {
	t.Helper()
	if want, err := wantValues.YAML(); err != nil {
		t.Fatal(err)
	} else if want != have {
		t.Fatalf("config values do not match: want\n%s\n\ngot\n%s\n", want, have)
	}
}

func Test_loadChart_mergesValues(t *testing.T) {
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
	as.Spec.Chart.Inline = kubetest.BuildInlineChart(t, ChartName /*template=*/, "", `
foo1:
  baz1: "hello"
bar1: 3`)
	wantValues := chartutil.Values{
		"bar1": 4,
		"bar2": chartutil.Values{"baz2": "test"},
		"foo1": chartutil.Values{"baz1": "hello"},
	}

	_, vals, err := loadChart(&as.Spec.Chart)
	if err != nil {
		t.Fatal(err)
	}
	verifyValues(t, vals, wantValues)
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

		// Now worker is busy. ensureUpdated should return false.
		started := rs.ensureUpdated(as)
		if started {
			t.Error("Expected ensureUpdated to return false because worker is busy")
		}

		close(unblockChan)
	})
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
