package v1

import (
	"bytes"
	"context"
	"io"
	"net/http"
	"os"
	"path"
	"testing"

	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/app"
	"github.com/googlecloudrobotics/core/src/go/cmd/token-vendor/repository/cloudiot"
)

const testDataPath = "testdata/cloudiot"

type isValidDeviceIDTest struct {
	deviceId string
	ret      bool
}

func TestValidateDeviceId(t *testing.T) {

	var isValidDeviceIDTests = []isValidDeviceIDTest{
		// invalid
		{"", false}, {`\\\\\\\\\\\\\`, false}, {"t\nt", false}, {"1", false},
		{"robot-dev-\ndevice", false}, {"1test", false},
		{"AAAAAAAAAAAAAAAAAAAarobot-dev-device-\neuwest1-test-com", false},
		// valid
		{"robot-dev-device", true}, {"TEST.com", true},
	}

	for _, test := range isValidDeviceIDTests {
		v := isValidDeviceID(test.deviceId)
		if v != test.ret {
			t.Errorf("isValidDeviceID(%q), got %v, want %v", test.deviceId, v, test.ret)
		}
	}
}

type RoundTripFunc func(req *http.Request) *http.Response

func (f RoundTripFunc) RoundTrip(req *http.Request) (*http.Response, error) {
	return f(req), nil
}

func NewTestHTTPClient(fn RoundTripFunc) *http.Client {
	return &http.Client{Transport: fn}
}

type publicKeyReadHandlerTest struct {
	desc           string
	isCalledUrl    string
	isResponseBody io.ReadCloser
	isKey          string
}

func mustRespBodyFromFile(t *testing.T, file string) io.ReadCloser {

	isResponseBody, err := os.ReadFile(file)
	if err != nil {
		t.Fatal(err.Error())
	}
	return io.NopCloser(bytes.NewBuffer(isResponseBody))
}

// Tests the publicKeyReadHandler handler with the IoT repository using a test http.Client
//
// Only the happy path of receiving a single key or no key is tested right now.
func TestPublicKeyReadHandlerWithIoT(t *testing.T) {

	var cases = []publicKeyReadHandlerTest{
		{
			"happy_path",
			"https://cloudiot.googleapis.com/v1/projects/testproject/locations/testregion/registries/testregistry/devices/testid?alt=json&fieldMask=credentials%2Cblocked&prettyPrint=false",
			mustRespBodyFromFile(t, path.Join(testDataPath, "describe_device.json")),
			"a_public_key",
		},
		{
			"happy_path_expired_key",
			"https://cloudiot.googleapis.com/v1/projects/testproject/locations/testregion/registries/testregistry/devices/testid?alt=json&fieldMask=credentials%2Cblocked&prettyPrint=false",
			mustRespBodyFromFile(t, path.Join(testDataPath, "describe_device_expired_key.json")),
			"",
		},
	}

	for _, test := range cases {
		t.Run(test.desc, func(t *testing.T) {
			runPublicKeyReadHandlerWithIoTCase(t, &test)
		})
	}

}

func runPublicKeyReadHandlerWithIoTCase(t *testing.T, test *publicKeyReadHandlerTest) {

	apiResponse := func(req *http.Request) *http.Response {
		gotCalledUrl := req.URL.String()
		if gotCalledUrl != test.isCalledUrl {
			t.Errorf("request URL missmatch, is %q, got %q", gotCalledUrl, test.isCalledUrl)
		}
		return &http.Response{
			StatusCode: http.StatusOK,
			Body:       test.isResponseBody,
			Header:     make(http.Header),
		}
	}
	client := NewTestHTTPClient(apiResponse)
	reg := cloudiot.Registry{Project: "testproject", Region: "testregion", Registry: "testregistry"}
	r, err := cloudiot.NewCloudIoTRepository(context.TODO(), reg, client)
	if err != nil {
		t.Fatal(err.Error())
	}
	tv, err := app.NewTokenVendor(context.TODO(), r)
	if err != nil {
		t.Fatal(err.Error())
	}
	key, err := tv.ReadPublicKey(context.TODO(), "testid")
	if err != nil {
		t.Errorf(err.Error())
	}
	if key != test.isKey {
		t.Errorf("public key response does not match with the test data, is %q, got %q",
			test.isKey, key)
	}
}
