package cloudiot

import (
	"reflect"
	"sort"
	"testing"

	iot "google.golang.org/api/cloudiot/v1"
)

type isExpiredTest struct {
	desc        string
	cred        iot.DeviceCredential
	expired     bool
	shouldError bool
}

func TestIsExpired(t *testing.T) {
	var cases = []isExpiredTest{
		{"empty credentials", iot.DeviceCredential{}, true, true},
		// The API returns 1970-01-01T00:00:00Z for credentials with no expiration set
		{"never expiring", iot.DeviceCredential{ExpirationTime: "1970-01-01T00:00:00Z"}, false, false},
		{"expired", iot.DeviceCredential{ExpirationTime: "1970-01-01T00:00:01Z"}, true, false},
		{"far in the future", iot.DeviceCredential{ExpirationTime: "2100-01-01T00:00:00Z"}, false, false},
		{"invalid expiration date", iot.DeviceCredential{ExpirationTime: "invalid date"}, true, true},
	}
	for _, test := range cases {
		t.Run(test.desc, func(t *testing.T) {
			expired, err := isExpired(&test.cred)
			if expired != test.expired {
				t.Errorf("expired is %v, got %v", test.expired, expired)
			}
			if (err == nil && test.shouldError) || (err != nil && !test.shouldError) {
				t.Errorf("error got %v, but is %v", err != nil, test.shouldError)
			}
		})
	}
}

type extractValidKeysTest struct {
	desc   string
	device *iot.Device
	keys   []string
}

func TestExtractValidKeys(t *testing.T) {
	newTestDevice := func(e string) *iot.Device {
		return &iot.Device{Credentials: []*iot.DeviceCredential{{ExpirationTime: e}}}
	}
	var extractValidKeysTests = []extractValidKeysTest{
		{"no keys", &iot.Device{}, []string{}},
		{"one expired, no key", newTestDevice("1970-01-01T00:00:00Z"), []string{}},
		{"one not expired, no key", newTestDevice("2030-01-01T00:00:00Z"), []string{}},
		{"one expired, with key",
			&iot.Device{Credentials: []*iot.DeviceCredential{
				{ExpirationTime: "2030-01-01T00:00:00Z",
					PublicKey: &iot.PublicKeyCredential{Key: "a valid key"}},
			}}, []string{"a valid key"}},
		{"one not expired, one expired, both with keys",
			&iot.Device{Credentials: []*iot.DeviceCredential{
				{ExpirationTime: "2030-01-01T00:00:00Z",
					PublicKey: &iot.PublicKeyCredential{Key: "a valid key"}},
				{ExpirationTime: "2001-01-01T00:00:00Z",
					PublicKey: &iot.PublicKeyCredential{Key: "an expired key"}}},
			}, []string{"a valid key"},
		},
	}
	for _, test := range extractValidKeysTests {
		t.Run(test.desc, func(t *testing.T) {
			keysGot := extractValidKeys(test.device)
			sort.Strings(keysGot)
			sort.Strings(test.keys)
			if !reflect.DeepEqual(keysGot, test.keys) {
				t.Errorf("got %v != is %v", keysGot, test.keys)
			}
		})
	}
}

func TestDevicePath(t *testing.T) {
	iotr := CloudIoTRepository{
		r: Registry{Project: "testProjectID", Region: "testRegion", Registry: "testRegistryID"},
	}
	is := "projects/testProjectID/locations/testRegion/registries/testRegistryID/devices/testDevice"
	got := iotr.devicePath("testDevice")
	if is != got {
		t.Errorf("is %s != got %s", is, got)
	}
}

func TestRegistryPath(t *testing.T) {
	iotr := CloudIoTRepository{
		r: Registry{Project: "testProjectID", Region: "testRegion", Registry: "testRegistryID"},
	}
	is := "projects/testProjectID/locations/testRegion/registries/testRegistryID"
	got := iotr.registryPath()
	if is != got {
		t.Errorf("is %s != got %s", is, got)
	}
}
