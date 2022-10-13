package main

import (
	"context"
	"io"
	"net/http"
	"os"
	"sort"
	"strings"
	"testing"

	"github.com/google/go-cmp/cmp"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/repository/cloudiot"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/repository/k8s"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/client-go/kubernetes/fake"
)

func mustFileToString(t *testing.T, name string) string {
	fp, err := os.Open(name)
	if err != nil {
		t.Fatal(err.Error())
	}
	bytes, err := io.ReadAll(fp)
	if err != nil {
		t.Fatalf(err.Error())
	}
	return string(bytes)
}

type RoundTripFunc func(req *http.Request) *http.Response

func (f RoundTripFunc) RoundTrip(req *http.Request) (*http.Response, error) {
	return f(req), nil
}

func NewTestHTTPClient(fn RoundTripFunc) *http.Client {
	return &http.Client{Transport: fn}
}

type MigrationTest struct {
	desc            string
	iotResponses    map[string]string // Cloud IoT: called URL -> response body file
	k8sIDsBefore    []string          // devices which exist on Kubernetes before migration
	wantK8sIDsAfter []string          // devices which should exist afterwards
	wantMigrated    int               // how many we expect to be migrated
}

func TestMigration(t *testing.T) {
	const iotBaseUrl = "https://cloudiot.googleapis.com/v1/projects/testproject/locations/testregion/registries/testregistry"
	var cases = []MigrationTest{
		{
			// simplest case, 2 on IoT, none on Kubernetes
			"none_on_k8s",
			map[string]string{
				iotBaseUrl + "/devices?alt=json&fieldMask=&prettyPrint=false":                                   "testdata/list_devices.json",
				iotBaseUrl + "/devices/testdevice-a?alt=json&fieldMask=credentials%2Cblocked&prettyPrint=false": "testdata/describe_device_a.json",
				iotBaseUrl + "/devices/testdevice-b?alt=json&fieldMask=credentials%2Cblocked&prettyPrint=false": "testdata/describe_device_b.json",
			},
			[]string{},
			[]string{"testdevice-a", "testdevice-b"},
			2,
		},
		// all exist already on Kubernetes
		{
			"all_on_k8s",
			map[string]string{
				iotBaseUrl + "/devices?alt=json&fieldMask=&prettyPrint=false":                                   "testdata/list_devices.json",
				iotBaseUrl + "/devices/testdevice-a?alt=json&fieldMask=credentials%2Cblocked&prettyPrint=false": "testdata/describe_device_a.json",
				iotBaseUrl + "/devices/testdevice-b?alt=json&fieldMask=credentials%2Cblocked&prettyPrint=false": "testdata/describe_device_b.json",
			},
			[]string{"testdevice-a", "testdevice-b"},
			[]string{"testdevice-a", "testdevice-b"},
			0,
		},
		// one new, one exists already, one additional on K8s
		{
			"one_new_one_exists_one_k8s",
			map[string]string{
				iotBaseUrl + "/devices?alt=json&fieldMask=&prettyPrint=false":                                   "testdata/list_devices.json",
				iotBaseUrl + "/devices/testdevice-a?alt=json&fieldMask=credentials%2Cblocked&prettyPrint=false": "testdata/describe_device_a.json",
				iotBaseUrl + "/devices/testdevice-b?alt=json&fieldMask=credentials%2Cblocked&prettyPrint=false": "testdata/describe_device_b.json",
			},
			[]string{"testdevice-a", "testdevice-c"},
			[]string{"testdevice-a", "testdevice-b", "testdevice-c"},
			1,
		},
		// one new, one is blocked
		{
			"one_new_one_blocked",
			map[string]string{
				iotBaseUrl + "/devices?alt=json&fieldMask=&prettyPrint=false":                                   "testdata/list_devices.json",
				iotBaseUrl + "/devices/testdevice-a?alt=json&fieldMask=credentials%2Cblocked&prettyPrint=false": "testdata/describe_device_a.json",
				iotBaseUrl + "/devices/testdevice-b?alt=json&fieldMask=credentials%2Cblocked&prettyPrint=false": "testdata/describe_device_b_blocked.json",
			},
			[]string{},
			[]string{"testdevice-a"},
			1,
		},
	}
	for _, test := range cases {
		t.Run(test.desc, func(t *testing.T) {
			runMigrationTest(t, &test)
		})
	}
}

func runMigrationTest(t *testing.T, test *MigrationTest) {
	// setup fake Cloud IoT
	fakeIoTHandler := func(req *http.Request) *http.Response {
		gotCalledUrl := req.URL.String()
		respFile, found := test.iotResponses[gotCalledUrl]
		if !found {
			t.Fatalf("request URL %q not known", gotCalledUrl)
		}
		return &http.Response{
			StatusCode: http.StatusOK,
			Body:       io.NopCloser(strings.NewReader(mustFileToString(t, respFile))),
			Header:     make(http.Header),
		}
	}
	client := NewTestHTTPClient(fakeIoTHandler)
	reg := cloudiot.Registry{Project: "testproject", Region: "testregion", Registry: "testregistry"}
	c, err := cloudiot.NewCloudIoTRepository(context.TODO(), reg, client)
	if err != nil {
		t.Fatal(err.Error())
	}
	// setup fake Kubernetes with prepopulated devices
	cs := fake.NewSimpleClientset()
	for _, id := range test.k8sIDsBefore {
		cm := &corev1.ConfigMap{
			TypeMeta: metav1.TypeMeta{
				Kind:       "ConfigMap",
				APIVersion: "v1",
			},
			ObjectMeta: metav1.ObjectMeta{
				Name: id,
			},
			Data: map[string]string{"pubKey": "anything"},
		}
		if _, err := cs.CoreV1().ConfigMaps("default").Create(context.TODO(), cm, metav1.CreateOptions{}); err != nil {
			t.Fatal(err)
		}
		if err != nil {
			t.Fatal(err)
		}
	}
	k, err := k8s.NewK8sRepository(context.TODO(), cs, "default")
	if err != nil {
		t.Fatal(err)
	}
	// run migration
	s, err := migrate(context.TODO(), c, k)
	if err != nil {
		t.Fatal(err)
	}
	// check Kubernetes backend and statistics
	if s.migrated != test.wantMigrated {
		t.Fatalf("got %d devices migrated, want %d", s.migrated, test.wantMigrated)
	}
	lsK8s, err := k.ListAllDeviceIDs(context.TODO())
	sort.Strings(lsK8s)
	sort.Strings(test.wantK8sIDsAfter)
	if diff := cmp.Diff(lsK8s, test.wantK8sIDsAfter); diff != "" {
		t.Fatalf("Kubernetes backend devices got %v, want %v, diff %v",
			lsK8s, test.wantK8sIDsAfter, diff)
	}
}
